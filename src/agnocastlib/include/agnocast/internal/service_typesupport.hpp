// A generic service endpoint needs two independent typesupports, each valid only while its shared
// library stays loaded, hence the paired shared_ptr<SharedLibrary> in ServiceTsBundle. service_ts
// exists only for rcl_service_init()/rcl_client_init(), so the Agnocast-only endpoints, which
// exchange payloads through shared memory, ignore it.

#pragma once

#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rosidl_runtime_c/service_type_support_struct.h>

#include <memory>
#include <string>

namespace agnocast
{

struct ServiceTsBundle
{
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_introspection;
  const rosidl_service_type_support_t * service_ts{nullptr};
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members{nullptr};
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members{nullptr};
};

// Throws std::runtime_error if a typesupport library cannot be loaded or the service type is
// malformed.
ServiceTsBundle load_service_typesupport(const std::string & service_type);

}  // namespace agnocast
