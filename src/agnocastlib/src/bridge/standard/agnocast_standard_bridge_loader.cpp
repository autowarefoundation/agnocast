#include "agnocast/bridge/standard/agnocast_standard_bridge_loader.hpp"

#include "agnocast/agnocast_utils.hpp"
#include "agnocast/bridge/agnocast_bridge_utils.hpp"

#include <dlfcn.h>
#include <elf.h>
#include <link.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace agnocast
{

StandardBridgeLoader::StandardBridgeLoader(const rclcpp::Logger & logger) : logger_(logger)
{
}

StandardBridgeLoader::~StandardBridgeLoader()
{
  cached_factories_.clear();
}

std::shared_ptr<void> StandardBridgeLoader::create(
  const MqMsgBridge & req, const std::string & bridge_target_key,
  const rclcpp::Node::SharedPtr & node, const rclcpp::QoS & qos)
{
  auto [entry_func, lib_handle] = resolve_factory_function(req, bridge_target_key);

  if (entry_func == 0) {
    const char * err = dlerror();
    RCLCPP_ERROR(
      logger_, "Failed to resolve factory for '%s': %s", bridge_target_key.c_str(),
      err ? err : "Unknown error");
    return nullptr;
  }

  return create_bridge_instance(
    reinterpret_cast<BridgeFn>(entry_func), lib_handle, node, req.pubsub_target, qos);
}

std::shared_ptr<void> StandardBridgeLoader::create_service(
  const MqMsgBridge & req, const std::string & bridge_target_key,
  const rclcpp::Node::SharedPtr & node)
{
  auto [entry_func, lib_handle] = resolve_factory_function(req, bridge_target_key);

  if (entry_func == 0) {
    const char * err = dlerror();
    RCLCPP_ERROR(
      logger_, "Failed to resolve factory for '%s': %s", bridge_target_key.c_str(),
      err ? err : "Unknown error");
    return nullptr;
  }

  return create_service_bridge_instance(
    reinterpret_cast<ServiceBridgeFn>(entry_func), lib_handle, node, req.srv_target);
}

std::shared_ptr<void> StandardBridgeLoader::create_bridge_instance(
  BridgeFn entry_func, const std::shared_ptr<void> & lib_handle,
  const rclcpp::Node::SharedPtr & node, const PubsubBridgeTargetInfo & target,
  const rclcpp::QoS & qos)
{
  try {
    auto bridge_resource = entry_func(node, target, qos);
    if (!bridge_resource) {
      return nullptr;
    }

    if (lib_handle) {
      // Prevent library unload while bridge_resource is alive (aliasing constructor)
      using BundleType = std::pair<std::shared_ptr<void>, std::shared_ptr<void>>;
      auto bundle = std::make_shared<BundleType>(lib_handle, bridge_resource);
      return {bundle, bridge_resource.get()};
    }

    RCLCPP_ERROR(logger_, "Library handle is missing. Cannot ensure bridge lifetime safety.");
    return nullptr;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception in factory: %s", e.what());
    return nullptr;
  }
}

std::shared_ptr<void> StandardBridgeLoader::create_service_bridge_instance(
  ServiceBridgeFn entry_func, const std::shared_ptr<void> & lib_handle,
  const rclcpp::Node::SharedPtr & node, const ServiceBridgeTargetInfo & target)
{
  try {
    auto bridge_resource = entry_func(node, target);
    if (!bridge_resource) {
      return nullptr;
    }

    if (lib_handle) {
      // Prevent library unload while bridge_resource is alive (aliasing constructor)
      using BundleType = std::pair<std::shared_ptr<void>, std::shared_ptr<void>>;
      auto bundle = std::make_shared<BundleType>(lib_handle, bridge_resource);
      return {bundle, bridge_resource.get()};
    }

    RCLCPP_ERROR(logger_, "Library handle is missing. Cannot ensure bridge lifetime safety.");
    return nullptr;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception in service factory: %s", e.what());
    return nullptr;
  }
}

std::pair<void *, uintptr_t> StandardBridgeLoader::load_library(
  const char * lib_path, const char * symbol_name)
{
  void * handle = nullptr;

  if (std::strcmp(symbol_name, MAIN_EXECUTABLE_SYMBOL) == 0) {
    handle = dlopen(nullptr, RTLD_NOW);
  } else {
    handle = dlopen(lib_path, RTLD_NOW | RTLD_LOCAL);
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
  const MqMsgBridge & req, const std::string & bridge_target_key)
{
  if (auto it = cached_factories_.find(bridge_target_key); it != cached_factories_.end()) {
    // Return the cached pair of the factory function and the shared library handle.
    return it->second;
  }

  // Clear any existing dynamic linker error state before loading the library and resolving the
  // symbol. This ensures that a subsequent call to dlerror() will report only errors that occurred
  // after this point.
  dlerror();
  auto [raw_handle, base_addr] = load_library(
    static_cast<const char *>(req.factory.shared_lib_path),
    static_cast<const char *>(req.factory.symbol_name));

  if ((raw_handle == nullptr) || (base_addr == 0)) {
    if (raw_handle != nullptr) {
      dlclose(raw_handle);
    }
    return {0, nullptr};
  }

  // Manage handle lifecycle
  std::shared_ptr<void> lib_handle_ptr(raw_handle, [](void * h) {
    if (h != nullptr) {
      dlclose(h);
    }
  });

  // Resolve main function
  uintptr_t entry_func = base_addr + req.factory.fn_offset;
  if (!is_address_in_library_code_segment(raw_handle, entry_func)) {
    RCLCPP_ERROR(
      logger_, "Main factory function pointer for '%s' is out of bounds: 0x%lx",
      bridge_target_key.c_str(), static_cast<unsigned long>(entry_func));
    return {0, nullptr};
  }

  // Service bridges do not have reverse factories yet, so early return here.
  if (req.is_service) {
    cached_factories_[bridge_target_key] = {entry_func, lib_handle_ptr};
    return {entry_func, lib_handle_ptr};
  }

  // Construct reverse key
  std::string_view suffix =
    (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) ? SUFFIX_A2R : SUFFIX_R2A;
  std::string reverse_key(static_cast<const char *>(req.pubsub_target.topic_name));
  reverse_key += suffix;

  uintptr_t reverse_func = base_addr + req.factory.fn_offset_reverse;
  if (!is_address_in_library_code_segment(raw_handle, reverse_func)) {
    RCLCPP_ERROR(
      logger_, "Reverse function pointer for '%s' is out of bounds: 0x%lx", reverse_key.c_str(),
      static_cast<unsigned long>(reverse_func));
    return {0, nullptr};
  }

  cached_factories_[bridge_target_key] = {entry_func, lib_handle_ptr};
  cached_factories_[reverse_key] = {reverse_func, lib_handle_ptr};

  return {entry_func, lib_handle_ptr};
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
