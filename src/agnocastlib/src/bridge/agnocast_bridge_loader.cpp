#include "agnocast/bridge/agnocast_bridge_loader.hpp"

#include "agnocast/agnocast_utils.hpp"

#include <dlfcn.h>
#include <elf.h>
#include <link.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace agnocast
{

BridgeLoader::BridgeLoader(const rclcpp::Logger & logger) : logger_(logger)
{
}

BridgeLoader::~BridgeLoader()
{
  cached_factories_.clear();
}

std::shared_ptr<void> BridgeLoader::create(
  const MqMsgBridge & req, const std::string & topic_name_with_direction,
  const rclcpp::Node::SharedPtr & node, const rclcpp::QoS & qos)
{
  auto [entry_func, lib_handle] = resolve_factory_function(req, topic_name_with_direction);

  if (entry_func == nullptr) {
    const char * err = dlerror();
    RCLCPP_ERROR(
      logger_, "Failed to resolve factory for '%s': %s", topic_name_with_direction.c_str(),
      err ? err : "Unknown error");
    return nullptr;
  }

  return create_bridge_instance(entry_func, lib_handle, node, req.target, qos);
}

std::shared_ptr<void> BridgeLoader::create_bridge_instance(
  BridgeFn entry_func, const std::shared_ptr<void> & lib_handle,
  const rclcpp::Node::SharedPtr & node, const BridgeTargetInfo & target, const rclcpp::QoS & qos)
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

std::pair<void *, uintptr_t> BridgeLoader::load_library(
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

std::pair<BridgeFn, std::shared_ptr<void>> BridgeLoader::resolve_factory_function(
  const MqMsgBridge & req, const std::string & topic_name_with_direction)
{
  if (auto it = cached_factories_.find(topic_name_with_direction); it != cached_factories_.end()) {
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
    return {nullptr, nullptr};
  }

  // Manage Handle Lifecycle
  std::shared_ptr<void> lib_handle_ptr(raw_handle, [](void * h) {
    if (h != nullptr) {
      dlclose(h);
    }
  });

  // Resolve Main Function
  uintptr_t entry_addr = base_addr + req.factory.fn_offset;
  BridgeFn entry_func = nullptr;

  if (is_address_in_library_code_segment(raw_handle, entry_addr)) {
    entry_func = reinterpret_cast<BridgeFn>(entry_addr);
  } else {
    RCLCPP_ERROR(
      logger_, "Main factory function pointer for '%s' is out of bounds: 0x%lx",
      topic_name_with_direction.c_str(), static_cast<unsigned long>(entry_addr));
    return {nullptr, nullptr};
  }

  // Register Reverse Function
  std::string_view suffix =
    (req.direction == BridgeDirection::ROS2_TO_AGNOCAST) ? SUFFIX_A2R : SUFFIX_R2A;
  std::string topic_name_with_reverse(static_cast<const char *>(req.target.topic_name));
  topic_name_with_reverse += suffix;

  uintptr_t reverse_addr = base_addr + req.factory.fn_offset_reverse;
  BridgeFn reverse_func = nullptr;

  if (is_address_in_library_code_segment(raw_handle, reverse_addr)) {
    reverse_func = reinterpret_cast<BridgeFn>(reverse_addr);
  } else {
    RCLCPP_ERROR(
      logger_, "Reverse function pointer for '%s' is out of bounds: 0x%lx",
      topic_name_with_reverse.c_str(), static_cast<unsigned long>(reverse_addr));
    return {nullptr, nullptr};
  }

  cached_factories_[topic_name_with_direction] = {entry_func, lib_handle_ptr};
  cached_factories_[topic_name_with_reverse] = {reverse_func, lib_handle_ptr};

  return {entry_func, lib_handle_ptr};
}

bool BridgeLoader::is_address_in_library_code_segment(void * handle, uintptr_t addr)
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
