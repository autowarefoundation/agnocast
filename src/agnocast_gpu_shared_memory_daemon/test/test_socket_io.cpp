// Unit tests for framed socket I/O, exercised over a socketpair (no accept loop).
#include "agnocast_gpu_shared_memory_daemon/socket_io.hpp"

#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <vector>

namespace proto = agnocast::gpu_shared_memory_daemon;

namespace
{

class SocketPair : public ::testing::Test
{
protected:
  void SetUp() override { ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, fds_), 0); }
  void TearDown() override
  {
    if (fds_[0] >= 0) {
      ::close(fds_[0]);
    }
    if (fds_[1] >= 0) {
      ::close(fds_[1]);
    }
  }

  int fds_[2] = {-1, -1};
};

}  // namespace

TEST_F(SocketPair, RoundTripFramedMessage)
{
  std::vector<std::uint8_t> payload = {1, 2, 3, 4, 5};
  ASSERT_TRUE(proto::write_message(fds_[0], proto::MessageType::kFreeRequest, payload));

  proto::MessageHeader header;
  std::vector<std::uint8_t> received;
  ASSERT_TRUE(proto::read_message(fds_[1], header, received));
  EXPECT_TRUE(proto::header_is_valid(header));
  EXPECT_EQ(header.type, static_cast<std::uint32_t>(proto::MessageType::kFreeRequest));
  EXPECT_EQ(received, payload);
}

TEST_F(SocketPair, RoundTripEmptyPayload)
{
  ASSERT_TRUE(proto::write_message(fds_[0], proto::MessageType::kListRequest, {}));

  proto::MessageHeader header;
  std::vector<std::uint8_t> received;
  ASSERT_TRUE(proto::read_message(fds_[1], header, received));
  EXPECT_EQ(header.type, static_cast<std::uint32_t>(proto::MessageType::kListRequest));
  EXPECT_TRUE(received.empty());
}

TEST_F(SocketPair, ReadReturnsFalseOnPeerClose)
{
  ::close(fds_[0]);
  fds_[0] = -1;

  proto::MessageHeader header;
  std::vector<std::uint8_t> received;
  EXPECT_FALSE(proto::read_message(fds_[1], header, received));
}

TEST_F(SocketPair, ReadRejectsInvalidHeaderMagic)
{
  // 16 bytes of garbage: header fails the magic/version check.
  std::vector<std::uint8_t> garbage(proto::kHeaderWireSize, 0xcc);
  ASSERT_EQ(
    ::send(fds_[0], garbage.data(), garbage.size(), 0), static_cast<ssize_t>(garbage.size()));

  proto::MessageHeader header;
  std::vector<std::uint8_t> received;
  EXPECT_FALSE(proto::read_message(fds_[1], header, received));
}

TEST_F(SocketPair, ReadRejectsOversizePayload)
{
  // Craft a valid header that declares a payload larger than kMaxPayloadSize.
  proto::MessageHeader header;
  header.type = static_cast<std::uint32_t>(proto::MessageType::kListResponse);
  header.payload_size = proto::kMaxPayloadSize + 1u;
  const auto header_bytes = proto::serialize_header(header);
  ASSERT_EQ(
    ::send(fds_[0], header_bytes.data(), header_bytes.size(), 0),
    static_cast<ssize_t>(header_bytes.size()));

  proto::MessageHeader out_header;
  std::vector<std::uint8_t> received;
  EXPECT_FALSE(proto::read_message(fds_[1], out_header, received));
}

TEST_F(SocketPair, WaitReadableTimesOutThenReadyOnData)
{
  EXPECT_EQ(proto::wait_readable(fds_[1], 10), 0);  // nothing sent yet

  ASSERT_TRUE(proto::write_message(fds_[0], proto::MessageType::kListRequest, {}));
  EXPECT_GT(proto::wait_readable(fds_[1], 1000), 0);
}
