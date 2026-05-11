#include "agnocast/bridge/standard/agnocast_standard_bridge_loader.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <dlfcn.h>
#include <elf.h>
#include <link.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace agnocast
{

StandardBridgeLoader::StandardBridgeLoader(
  rclcpp::Node::SharedPtr container_node, const rclcpp::Logger & logger)
: container_node_(std::move(container_node)), logger_(logger)
{
}

StandardBridgeLoader::~StandardBridgeLoader()
{
  cached_factories_.clear();
}

std::shared_ptr<PubsubBridgeBase> StandardBridgeLoader::start_pubsub_bridge(
  const std::string & topic_name, BridgeDirection direction, const BridgeFactorySpec & factory_spec,
  const rclcpp::QoS & qos)
{
  auto [entry_func, lib_handle] =
    resolve_factory_function(topic_name, direction, factory_spec, false);

  if (entry_func == 0) {
    const char * err = dlerror();
    RCLCPP_ERROR(
      logger_, "Failed to resolve %s factory for '%s': %s",
      (direction == BridgeDirection::ROS2_TO_AGNOCAST) ? "R2A" : "A2R", topic_name.c_str(),
      err ? err : "Unknown error");
    return nullptr;
  }

  return create_bridge_instance<PubsubBridgeBase, PubsubBridgeFn>(
    reinterpret_cast<PubsubBridgeFn>(entry_func), lib_handle, topic_name, qos);
}

std::shared_ptr<ServiceBridgeBase> StandardBridgeLoader::start_service_bridge(
  const std::string & service_name, BridgeDirection direction,
  const BridgeFactorySpec & factory_spec, const rclcpp::QoS & qos)
{
  auto [entry_func, lib_handle] =
    resolve_factory_function(service_name, direction, factory_spec, true);

  if (entry_func == 0) {
    const char * err = dlerror();
    RCLCPP_ERROR(
      logger_, "Failed to resolve %s service factory for '%s': %s",
      (direction == BridgeDirection::ROS2_TO_AGNOCAST) ? "R2A" : "A2R", service_name.c_str(),
      err ? err : "Unknown error");
    return nullptr;
  }

  return create_bridge_instance<ServiceBridgeBase, ServiceBridgeFn>(
    reinterpret_cast<ServiceBridgeFn>(entry_func), lib_handle, service_name, qos);
}

template <typename BridgeBaseT, typename BridgeFnT>
std::shared_ptr<BridgeBaseT> StandardBridgeLoader::create_bridge_instance(
  BridgeFnT entry_func, const std::shared_ptr<void> & lib_handle, const std::string & name,
  const rclcpp::QoS & qos)
{
  try {
    auto bridge_resource = entry_func(container_node_, name, qos);
    if (!bridge_resource) {
      return nullptr;
    }

    if (lib_handle) {
      // Prevent library unload while bridge_resource is alive (aliasing constructor)
      using BundleType = std::pair<std::shared_ptr<void>, std::shared_ptr<BridgeBaseT>>;
      auto bundle = std::make_shared<BundleType>(lib_handle, bridge_resource);
      return {bundle, bridge_resource.get()};
    }

    RCLCPP_ERROR(logger_, "Library handle is missing. Cannot ensure bridge lifetime safety.");
    return nullptr;

  } catch (const std::exception & e) {
    if constexpr (std::is_same_v<BridgeBaseT, ServiceBridgeBase>) {
      RCLCPP_ERROR(logger_, "Exception in service factory: %s", e.what());
    } else {
      RCLCPP_ERROR(logger_, "Exception in pubsub factory: %s", e.what());
    }
    return nullptr;
  }
}

std::pair<void *, uintptr_t> StandardBridgeLoader::load_library(
  const std::optional<std::string> & shared_lib_path)
{
  void * handle = nullptr;

  if (!shared_lib_path.has_value()) {
    handle = dlopen(nullptr, RTLD_NOW);
  } else {
    handle = dlopen(shared_lib_path.value().c_str(), RTLD_NOW | RTLD_LOCAL);
  }

  if (handle == nullptr) {
    return {nullptr, 0};
  }

  struct link_map * map = nullptr;
  if (dlinfo(handle, RTLD_DI_LINKMAP, &map) != 0) {
    dlclose(handle);
    return {nullptr, 0};
  }
  return {handle, map->l_addr};
}

std::pair<uintptr_t, std::shared_ptr<void>> StandardBridgeLoader::resolve_factory_function(
  const std::string & name, BridgeDirection direction, const BridgeFactorySpec & factory_spec,
  bool is_service)
{
  std::string key_r2a = name;
  key_r2a += is_service ? SUFFIX_SERVICE_R2A : SUFFIX_PUBSUB_R2A;
  std::string key_a2r = name;
  key_a2r += is_service ? SUFFIX_SERVICE_A2R : SUFFIX_PUBSUB_A2R;

  const bool is_r2a = (direction == BridgeDirection::ROS2_TO_AGNOCAST);

  if (auto it = cached_factories_.find(is_r2a ? key_r2a : key_a2r); it != cached_factories_.end()) {
    // Return the cached pair of the factory function and the shared library handle.
    return it->second;
  }

  // Clear any existing dynamic linker error state before loading the library and resolving the
  // symbol. This ensures that a subsequent call to dlerror() will report only errors that occurred
  // after this point.
  dlerror();
  auto [raw_handle, base_addr] = load_library(factory_spec.shared_lib_path);

  if (raw_handle == nullptr) {
    return {0, nullptr};
  }

  // Manage Handle Lifecycle
  std::shared_ptr<void> lib_handle_ptr(raw_handle, [](void * h) {
    if (h != nullptr) {
      dlclose(h);
    }
  });

  // Add R2A function.
  uintptr_t addr_r2a = base_addr + factory_spec.fn_offset_r2a;
  if (!is_address_in_library_code_segment(raw_handle, addr_r2a)) {
    RCLCPP_ERROR(
      logger_, "R2A factory function pointer for '%s' is out of bounds: 0x%lx", key_r2a.c_str(),
      static_cast<unsigned long>(addr_r2a));
    return {0, nullptr};
  }
  cached_factories_[key_r2a] = {addr_r2a, lib_handle_ptr};

  // Add A2R function.
  uintptr_t addr_a2r = base_addr + factory_spec.fn_offset_a2r;
  if (!is_address_in_library_code_segment(raw_handle, addr_a2r)) {
    RCLCPP_ERROR(
      logger_, "A2R factory function pointer for '%s' is out of bounds: 0x%lx", key_a2r.c_str(),
      static_cast<unsigned long>(addr_a2r));
    return {0, nullptr};
  }
  cached_factories_[key_a2r] = {addr_a2r, lib_handle_ptr};

  if (is_r2a) {
    return {addr_r2a, lib_handle_ptr};
  }
  return {addr_a2r, lib_handle_ptr};
}

bool StandardBridgeLoader::is_address_in_library_code_segment(void * handle, uintptr_t addr)
{
  struct link_map * lm = nullptr;
  if (dlinfo(handle, RTLD_DI_LINKMAP, &lm) != 0 || lm == nullptr) {
    return false;
  }

  const auto base = static_cast<uintptr_t>(lm->l_addr);
  const auto * ehdr = reinterpret_cast<const ElfW(Ehdr) *>(base);
  const auto * phdr = reinterpret_cast<const ElfW(Phdr) *>(base + ehdr->e_phoff);

  for (int i = 0; i < ehdr->e_phnum; ++i) {
    const auto & segment = phdr[i];  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const auto flags = segment.p_flags;
    constexpr auto exec_flag = static_cast<ElfW(Word)>(PF_X);

    if (segment.p_type == PT_LOAD && ((flags & exec_flag) != 0U)) {
      const uintptr_t seg_start = base + segment.p_vaddr;
      const uintptr_t seg_end = seg_start + segment.p_memsz;

      if (addr >= seg_start && addr < seg_end) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace agnocast
