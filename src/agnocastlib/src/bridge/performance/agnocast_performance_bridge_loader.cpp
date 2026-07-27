#include "agnocast/bridge/performance/agnocast_performance_bridge_loader.hpp"

#include "agnocast/agnocast_client.hpp"
#include "agnocast/bridge/agnocast_bridge_node.hpp"
#include "agnocast/vendor/rclcpp/generic_service.hpp"
#include "rclcpp/version.h"

#include <ament_index_cpp/get_package_prefix.hpp>

#include <dlfcn.h>

#include <algorithm>
#include <cstdlib>
#include <sstream>

namespace agnocast
{

namespace
{

/// @brief Returns a function that copies a ROS 2 message into Agnocast shared memory via CDR
/// serialization/deserialization. The function returns a bool indicating whether the copy was
/// successful.
///
/// The caller must ensure that the destination memory block is initialized with the introspection
/// init function.
std::function<bool(const ipc_shared_ptr<void> &, const std::shared_ptr<void> &)> get_copier(
  const std::string & message_type)
{
  // --- typesupport setup ---
  static const std::string ts_identifier = "rosidl_typesupport_cpp";

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib =
    rclcpp::get_typesupport_library(message_type, ts_identifier);

#if RCLCPP_VERSION_MAJOR >= 28
  const rosidl_message_type_support_t * ts_handle =
    rclcpp::get_message_typesupport_handle(message_type, ts_identifier, *ts_lib);
#else
  const rosidl_message_type_support_t * ts_handle =
    rclcpp::get_typesupport_handle(message_type, ts_identifier, *ts_lib);
#endif
  // -------------------------

  return [ts_lib = std::move(ts_lib), ts_handle](
           const ipc_shared_ptr<void> & agno_msg, const std::shared_ptr<void> & ros_msg) {
    (void)ts_lib;

    rclcpp::SerializedMessage serialized;
    if (
      rmw_serialize(ros_msg.get(), ts_handle, &serialized.get_rcl_serialized_message()) !=
      RMW_RET_OK) {
      return false;
    }
    if (
      rmw_deserialize(&serialized.get_rcl_serialized_message(), ts_handle, agno_msg.get()) !=
      RMW_RET_OK) {
      return false;
    }
    return true;
  };
}

}  // namespace

PerformanceBridgeLoader::PerformanceBridgeLoader(const rclcpp::Logger & logger) : logger_(logger)
{
}

PerformanceBridgeLoader::~PerformanceBridgeLoader()
{
  for (auto & pair : loaded_libraries_) {
    if (pair.second != nullptr) {
      dlclose(pair.second);
    }
  }
  loaded_libraries_.clear();
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_r2a_pubsub_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "create_r2a_pubsub_bridge", false);
  if (symbol == nullptr) {
    // Fall back to the generic bridge, which is independent of plugins.
    RCLCPP_DEBUG(
      logger_, "No plugin found for topic '%s' (type: %s). Using generic bridge.",
      topic_name.c_str(), message_type.c_str());
    return create_r2a_pubsub_bridge_generic(node, topic_name, message_type, qos);
  }

  auto factory = reinterpret_cast<R2APubsubBridgeFactory>(symbol);
  return factory(std::move(node), topic_name, qos);
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_a2r_pubsub_bridge(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const std::string & message_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(message_type, "create_a2r_pubsub_bridge", false);
  if (symbol == nullptr) {
    // Fall back to the generic bridge, which is independent of plugins.
    RCLCPP_DEBUG(
      logger_, "No plugin found for topic '%s' (type: %s). Using generic bridge.",
      topic_name.c_str(), message_type.c_str());
    return create_a2r_pubsub_bridge_generic(node, topic_name, message_type, qos);
  }

  auto factory = reinterpret_cast<A2RPubsubBridgeFactory>(symbol);
  return factory(std::move(node), topic_name, qos);
}

ServiceBridgeEntity PerformanceBridgeLoader::create_r2a_service_bridge(
  rclcpp::Node::SharedPtr node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(service_type, "create_r2a_service_bridge", true);
  if (symbol == nullptr) {
    // Fall back to generic bridge, which is independent of plugins.
    RCLCPP_DEBUG(
      logger_, "No plugin found for service '%s' (type: %s). Using generic bridge.",
      service_name.c_str(), service_type.c_str());
    return create_r2a_service_bridge_generic(node, service_name, service_type, qos);
  }

  auto factory = reinterpret_cast<R2AServiceBridgeFactory>(symbol);
  return factory(std::move(node), service_name, qos);
}

ServiceBridgeEntity PerformanceBridgeLoader::create_a2r_service_bridge(
  rclcpp::Node::SharedPtr node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos)
{
  void * symbol = get_bridge_factory_symbol(service_type, "create_a2r_service_bridge", true);
  if (symbol == nullptr) {
    return {nullptr, nullptr, nullptr};
  }

  auto factory = reinterpret_cast<A2RServiceBridgeFactory>(symbol);
  return factory(std::move(node), service_name, qos);
}

std::string PerformanceBridgeLoader::convert_type_to_snake_case(const std::string & message_type)
{
  std::string result = message_type;
  std::replace(result.begin(), result.end(), '/', '_');
  return result;
}

std::vector<std::string> PerformanceBridgeLoader::generate_library_paths()
{
  std::vector<std::string> paths;
  const std::string lib_name = "libagnocast_bridge_plugins.so";

  // 1. Check environment variable AGNOCAST_BRIDGE_PLUGINS_PATH (colon-separated)
  const char * env_path = std::getenv("AGNOCAST_BRIDGE_PLUGINS_PATH");
  if (env_path != nullptr) {
    std::string env_str(env_path);
    std::istringstream iss(env_str);
    std::string path;
    while (std::getline(iss, path, ':')) {
      if (!path.empty()) {
        std::string full_path;
        full_path.reserve(path.size() + 1 + lib_name.size());
        full_path += path;
        full_path += '/';
        full_path += lib_name;
        paths.push_back(std::move(full_path));
      }
    }
  }

  // 2. Check user-generated agnocast_bridge_plugins package
  try {
    const std::string user_prefix = ament_index_cpp::get_package_prefix("agnocast_bridge_plugins");
    paths.push_back(user_prefix + "/lib/agnocast_bridge_plugins/" + lib_name);
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    // Package not found, continue
  }

  return paths;
}

void * PerformanceBridgeLoader::load_library_from_paths(
  const std::vector<std::string> & paths, std::string & last_error)
{
  last_error.clear();

  if (paths.empty()) {
    return nullptr;
  }

  for (const auto & path : paths) {
    // Check cache first
    if (loaded_libraries_.find(path) != loaded_libraries_.end()) {
      return loaded_libraries_[path];
    }

    // Try to load
    void * handle = dlopen(path.c_str(), RTLD_LAZY);
    if (handle != nullptr) {
      loaded_libraries_[path] = handle;
      return handle;
    }
    // Capture error immediately before any subsequent call clears it.
    const char * err = dlerror();
    if (err != nullptr) {
      last_error = err;
    }
  }

  return nullptr;
}

void * PerformanceBridgeLoader::get_bridge_factory_symbol(
  const std::string & type_name, const std::string & symbol_name_prefix, bool is_service)
{
  // TODO(bdm-k): Remove the error paths for service bridges once we have an A2R generic service
  // bridge implementation.
  const char * type_label = is_service ? "service" : "message";
  std::string snake_type = convert_type_to_snake_case(type_name);
  std::vector<std::string> lib_paths = generate_library_paths();

  std::string last_dlopen_error;
  void * handle = load_library_from_paths(lib_paths, last_dlopen_error);
  if (handle == nullptr) {
    if (lib_paths.empty()) {
      if (is_service) {
        RCLCPP_ERROR(
          logger_,
          "No plugin paths available for service bridge. Have you generated bridge plugins?");
      }
    } else {
      std::string tried_paths;
      for (const auto & path : lib_paths) {
        tried_paths += "\n  - " + path;
      }
      if (is_service) {
        RCLCPP_ERROR(
          logger_,
          "Failed to load plugin for service type '%s'.\n"
          "Tried paths:%s\n"
          "Last error: %s",
          type_name.c_str(), tried_paths.c_str(), last_dlopen_error.c_str());
      } else {
        RCLCPP_WARN(
          logger_,
          "Failed to load plugin for message type '%s'. Falling back to generic bridge.\n"
          "Tried paths:%s\n"
          "Last error: %s",
          type_name.c_str(), tried_paths.c_str(), last_dlopen_error.c_str());
      }
    }
    return nullptr;
  }

  const std::string symbol_name = symbol_name_prefix + "_" + snake_type;

  dlerror();
  void * symbol = dlsym(handle, symbol_name.c_str());

  const char * dlsym_error = dlerror();
  if (dlsym_error != nullptr) {
    if (is_service) {
      RCLCPP_ERROR(
        logger_, "Failed to find symbol '%s' for %s type '%s': %s", symbol_name.c_str(), type_label,
        type_name.c_str(), dlsym_error);
    }
    return nullptr;
  }

  if (symbol == nullptr) {
    if (is_service) {
      RCLCPP_ERROR(
        logger_,
        "Symbol '%s' was found for %s type '%s' but returned NULL, which is invalid for a factory "
        "function.",
        symbol_name.c_str(), type_label, type_name.c_str());
    }
    return nullptr;
  }

  return symbol;
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_r2a_pubsub_bridge_generic(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name,
  const std::string & message_type, const rclcpp::QoS & qos)
{
  auto agno_pub = std::make_shared<agnocast::GenericPublisher>(
    node.get(), topic_name, message_type,
    rclcpp::QoS(agnocast::DEFAULT_QOS_DEPTH).transient_local(), agnocast::PublisherOptions{},
    agnocast::PublisherRole::BridgeInternal);

  // auto_add=false: the bridge manager adds this group to the executor explicitly, after the
  // subscription is created, so it is never classified before its subscription exists. Do not drop
  // the false here; with the default (true) the manager's explicit add_callback_group would abort.
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions opts;
  opts.ignore_local_publications = true;
  opts.callback_group = cb_group;

  // Compatibility note:
  // - Humble expects the callback taking std::shared_ptr<SerializedMessage>.
  // - Jazzy treats the callback as AnySubscriptionCallback, but if we receive the message as
  //   std::shared_ptr<SerializedMessage>, rclcpp deep-copies the message.
  // Keep the Humble/Jazzy split here for performance optimization.
  auto sub = node->create_generic_subscription(
    topic_name, message_type, qos,
#if RCLCPP_VERSION_MAJOR >= 28
    [agno_pub](const rclcpp::SerializedMessage & serialized_msg) {
      agno_pub->publish(serialized_msg);
    },
#else
    [agno_pub](const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg) {
      agno_pub->publish(*serialized_msg);
    },
#endif
    opts);

  PerformancePubsubBridgeResult result;
  result.entity_handle = sub;
  result.callback_group = cb_group;
  return result;
}

PerformancePubsubBridgeResult PerformanceBridgeLoader::create_a2r_pubsub_bridge_generic(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name,
  const std::string & message_type, const rclcpp::QoS & qos)
{
  auto ros_pub = node->create_generic_publisher(
    topic_name, message_type,
    rclcpp::QoS(agnocast::DEFAULT_QOS_DEPTH).reliable().transient_local());

  // auto_add=false: the bridge manager adds this group to the executor explicitly, after the
  // subscription is created, so it is never classified before its subscription exists. Do not drop
  // the false here; with the default (true) the manager's explicit add_callback_group would abort.
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  agnocast::SubscriptionOptions sub_opts;
  sub_opts.ignore_local_publications = true;
  sub_opts.callback_group = cb_group;

  auto agno_sub = std::make_shared<agnocast::GenericSubscription>(
    node.get(), topic_name, message_type, qos,
    [ros_pub](const rclcpp::SerializedMessage & serialized_msg) {
      ros_pub->publish(serialized_msg);
    },
    sub_opts, agnocast::SubscriptionRole::BridgeInternal);

  return {agno_sub, cb_group};
}

ServiceBridgeEntity PerformanceBridgeLoader::create_r2a_service_bridge_generic(
  const rclcpp::Node::SharedPtr & node, const std::string & service_name,
  const std::string & service_type, const rclcpp::QoS & qos)
{
  const std::string request_type = service_type + "_Request";

  auto srv_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto client_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto agno_client = std::make_shared<agnocast::GenericClient>(
    node.get(), service_name, service_type, qos, client_cb_group, ClientRole::AgnocastOnly);

  auto ros_srv = agnocast::vendor_rclcpp::GenericService::create_generic_service(
    node.get(), service_name, service_type,
    [agno_client = std::move(agno_client), request_copier = get_copier(request_type)](
      const std::shared_ptr<agnocast::vendor_rclcpp::GenericService> & service_handle,
      const std::shared_ptr<rmw_request_id_t> & request_header,
      const std::shared_ptr<void> & ros_req) {
      ipc_shared_ptr<void> agno_req = agno_client->borrow_loaned_request();

      // Copy the ROS 2 request into Agnocast shared memory (ipc_shared_ptr).
      if (!request_copier(agno_req, ros_req)) {
        RCLCPP_ERROR(
          logger,
          "Serialization error occurred in generic service bridge for '%s'; dropping request",
          service_handle->get_service_name());
        agno_client->cancel_request(std::move(agno_req));
        return;
      }

      agno_client->async_send_request(
        std::move(agno_req),
        [service_handle, request_header](const agnocast::GenericClient::SharedFuture & future) {
          const auto & agno_res = future.get();

          // Reuse Agnocast response payload directly as ROS response buffer.
          auto borrowed_ros_res = std::shared_ptr<void>(agno_res.get(), [](void *) { /* no-op */ });
          service_handle->send_response(*request_header, borrowed_ros_res);
        });
    },
    qos, srv_cb_group);

  return {ros_srv, srv_cb_group, client_cb_group};
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
}

}  // namespace agnocast
