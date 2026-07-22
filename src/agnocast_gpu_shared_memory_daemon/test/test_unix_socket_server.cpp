// End-to-end test of the Unix-domain-socket server over a real socket, driven by a
// client on the same host. Uses the mock backend, so it needs no GPU.
#include "gpu_shared_memory_pool.hpp"
#include "mock_slot_backend.hpp"
#include "socket_io.hpp"
#include "unix_socket_server.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace proto = agnocast::gpu_shared_memory_daemon;
using proto::test::MockSlotBackend;
using proto::test::two_class_config;

namespace
{

std::string temp_socket_path()
{
  return "/tmp/agnocast_gpu_daemon_test_" + std::to_string(::getpid()) + ".sock";
}

// Connects a client stream socket to `path`. Returns the fd, or -1 on failure.
int connect_client(const std::string & path)
{
  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  struct sockaddr_un addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

// Sends one request and reads back one response of the expected type.
bool round_trip(
  int fd, proto::MessageType request_type, const std::vector<std::uint8_t> & request_payload,
  proto::MessageType expected_response_type, std::vector<std::uint8_t> & response_payload)
{
  if (!proto::write_message(fd, request_type, request_payload)) {
    return false;
  }
  proto::MessageHeader header;
  if (!proto::read_message(fd, header, response_payload)) {
    return false;
  }
  return header.type == static_cast<std::uint32_t>(expected_response_type);
}

}  // namespace

class ServerFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(pool_.initialize(two_class_config()));
    socket_path_ = temp_socket_path();
    ::unlink(socket_path_.c_str());
    server_ = std::make_unique<proto::UnixSocketServer>(pool_, socket_path_);
    ASSERT_TRUE(server_->start());
    server_thread_ = std::thread([this] { server_->run(); });
  }

  void TearDown() override
  {
    server_->request_stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    server_.reset();
  }

  MockSlotBackend backend_;
  proto::GpuSharedMemoryPool pool_{backend_};
  std::string socket_path_;
  std::unique_ptr<proto::UnixSocketServer> server_;
  std::thread server_thread_;
};

TEST_F(ServerFixture, HandshakeListAllocFreeOverSocket)
{
  const int fd = connect_client(socket_path_);
  ASSERT_GE(fd, 0);

  // Handshake.
  std::vector<std::uint8_t> payload;
  ASSERT_TRUE(round_trip(
    fd, proto::MessageType::kHandshakeRequest, {}, proto::MessageType::kHandshakeResponse,
    payload));
  proto::HandshakeResponse handshake;
  ASSERT_TRUE(proto::deserialize_handshake_response(payload.data(), payload.size(), &handshake));
  EXPECT_EQ(handshake.backend_type, static_cast<std::uint32_t>(proto::BackendType::kCudaIpc));
  EXPECT_EQ(handshake.gpu_uuid, "GPU-mock-0001");

  // List.
  ASSERT_TRUE(round_trip(
    fd, proto::MessageType::kListRequest, {}, proto::MessageType::kListResponse, payload));
  proto::ListResponse list;
  ASSERT_TRUE(proto::deserialize_list_response(payload.data(), payload.size(), &list));
  EXPECT_EQ(list.slots.size(), 5u);

  // Alloc.
  proto::AllocRequest alloc_request;
  alloc_request.size = 1000;
  ASSERT_TRUE(round_trip(
    fd, proto::MessageType::kAllocRequest, proto::serialize_alloc_request(alloc_request),
    proto::MessageType::kAllocResponse, payload));
  proto::AllocResponse alloc_response;
  ASSERT_TRUE(proto::deserialize_alloc_response(payload.data(), payload.size(), &alloc_response));
  ASSERT_EQ(alloc_response.status, static_cast<std::uint32_t>(proto::Status::kOk));
  EXPECT_EQ(pool_.free_slot_count(), 4u);

  // Free.
  proto::FreeRequest free_request;
  free_request.slot_id = alloc_response.slot_id;
  ASSERT_TRUE(round_trip(
    fd, proto::MessageType::kFreeRequest, proto::serialize_free_request(free_request),
    proto::MessageType::kFreeResponse, payload));
  proto::FreeResponse free_response;
  ASSERT_TRUE(proto::deserialize_free_response(payload.data(), payload.size(), &free_response));
  EXPECT_EQ(free_response.status, static_cast<std::uint32_t>(proto::Status::kOk));
  EXPECT_EQ(pool_.free_slot_count(), 5u);

  ::close(fd);
}

TEST_F(ServerFixture, TwoClientsShareTheSamePool)
{
  const int fd_a = connect_client(socket_path_);
  const int fd_b = connect_client(socket_path_);
  ASSERT_GE(fd_a, 0);
  ASSERT_GE(fd_b, 0);

  // Exhaust the whole pool (5 slots) from client A.
  std::vector<std::uint8_t> payload;
  for (int i = 0; i < 5; ++i) {
    proto::AllocRequest request;
    request.size = 100;
    ASSERT_TRUE(round_trip(
      fd_a, proto::MessageType::kAllocRequest, proto::serialize_alloc_request(request),
      proto::MessageType::kAllocResponse, payload));
    proto::AllocResponse response;
    ASSERT_TRUE(proto::deserialize_alloc_response(payload.data(), payload.size(), &response));
    ASSERT_EQ(response.status, static_cast<std::uint32_t>(proto::Status::kOk));
  }

  // Client B now sees no free slot (shared pool) and is told immediately.
  proto::AllocRequest request;
  request.size = 100;
  ASSERT_TRUE(round_trip(
    fd_b, proto::MessageType::kAllocRequest, proto::serialize_alloc_request(request),
    proto::MessageType::kAllocResponse, payload));
  proto::AllocResponse response;
  ASSERT_TRUE(proto::deserialize_alloc_response(payload.data(), payload.size(), &response));
  EXPECT_EQ(response.status, static_cast<std::uint32_t>(proto::Status::kNoFreeSlot));

  ::close(fd_a);
  ::close(fd_b);
}
