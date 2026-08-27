#include "agnocast/internal/service_wire_type.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <gtest/gtest.h>

#include <cstddef>

// The typed and generic wrappers describe the same buffer from two directions:
// ServiceRequestWrapper<ServiceT> derives from ServiceT::Request, while GenericRequestWrapper
// locates the metadata at `buffer + offsetof_meta(size_of_)`. The static_asserts in
// service_wire_type.hpp only pin the total size, so the offsets themselves are checked here.

namespace
{

template <typename ServiceT>
void expect_request_layout_agrees()
{
  agnocast::ServiceRequestWrapper<ServiceT> wrapper;
  const auto * base = reinterpret_cast<const char *>(&wrapper);

  const auto * payload = static_cast<const typename ServiceT::Request *>(&wrapper);
  EXPECT_EQ(reinterpret_cast<const char *>(payload), base)
    << "the payload base must start the buffer: the generic path treats the buffer start as the "
       "payload";

  const auto * meta = static_cast<const agnocast::RequestMeta *>(&wrapper);
  EXPECT_EQ(
    reinterpret_cast<const char *>(meta) - base,
    static_cast<std::ptrdiff_t>(offsetof_meta(sizeof(typename ServiceT::Request))))
    << "metadata must sit where GenericRequestWrapper::get_meta_ptr computes it";
}

template <typename ServiceT>
void expect_response_layout_agrees()
{
  agnocast::ServiceResponseWrapper<ServiceT> wrapper;
  const auto * base = reinterpret_cast<const char *>(&wrapper);

  const auto * payload = static_cast<const typename ServiceT::Response *>(&wrapper);
  EXPECT_EQ(reinterpret_cast<const char *>(payload), base);

  const auto * meta = static_cast<const agnocast::ResponseMeta *>(&wrapper);
  EXPECT_EQ(
    reinterpret_cast<const char *>(meta) - base,
    static_cast<std::ptrdiff_t>(offsetof_meta(sizeof(typename ServiceT::Response))));
}

}  // namespace

TEST(ServiceWireTypeTest, RequestLayoutAgreesBetweenTypedAndGenericWrappers)
{
  expect_request_layout_agrees<std_srvs::srv::SetBool>();
}

TEST(ServiceWireTypeTest, ResponseLayoutAgreesBetweenTypedAndGenericWrappers)
{
  expect_response_layout_agrees<std_srvs::srv::SetBool>();
}

TEST(ServiceWireTypeTest, EmptyPayloadLayoutAgreesBetweenTypedAndGenericWrappers)
{
  expect_request_layout_agrees<std_srvs::srv::Empty>();
  expect_response_layout_agrees<std_srvs::srv::Empty>();
}
