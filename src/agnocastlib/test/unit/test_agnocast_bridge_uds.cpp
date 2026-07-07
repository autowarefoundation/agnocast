#include "agnocast/bridge/agnocast_bridge_uds.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

namespace
{

// Abstract-namespace address unique to this process + test-provided suffix, so
// parallel test binaries and repeated runs never collide with a live socket.
std::string make_unique_abstract_addr(const std::string & tag)
{
  std::string addr;
  addr.push_back('\0');
  addr += "agnocast_bridge_uds_test_";
  addr += std::to_string(getpid());
  addr += "_";
  addr += tag;
  return addr;
}

// RAII owner so a failed EXPECT doesn't leak the fd into later tests.
class ScopedFd
{
public:
  explicit ScopedFd(int fd) : fd_(fd) {}
  ~ScopedFd()
  {
    if (fd_ != -1) {
      close(fd_);
    }
  }
  ScopedFd(const ScopedFd &) = delete;
  ScopedFd & operator=(const ScopedFd &) = delete;
  int get() const { return fd_; }

private:
  int fd_;
};

}  // namespace

// ---- detail::fill_abstract_sockaddr ---------------------------------------

TEST(BridgeUdsAddrTest, EmptyAddressRejected)
{
  sockaddr_un sa{};
  EXPECT_THROW(agnocast::detail::fill_abstract_sockaddr("", sa), std::invalid_argument);
}

TEST(BridgeUdsAddrTest, NonNulPrefixedAddressRejected)
{
  sockaddr_un sa{};
  EXPECT_THROW(
    agnocast::detail::fill_abstract_sockaddr("agnocast_bridge_no_nul", sa), std::invalid_argument);
}

TEST(BridgeUdsAddrTest, OverlongAddressRejected)
{
  sockaddr_un sa{};
  // Leading NUL + (sizeof(sun_path)) bytes = one byte too many for sun_path.
  std::string too_long(sizeof(sa.sun_path) + 1, 'x');
  too_long[0] = '\0';
  EXPECT_THROW(agnocast::detail::fill_abstract_sockaddr(too_long, sa), std::length_error);
}

TEST(BridgeUdsAddrTest, ValidAddressPopulatesSockaddr)
{
  const std::string addr = std::string("\0hello", 6);  // NUL + "hello"

  sockaddr_un sa{};
  // Poison the padding to prove fill_abstract_sockaddr zeroes the whole struct.
  std::memset(&sa, 0xAB, sizeof(sa));

  const socklen_t alen = agnocast::detail::fill_abstract_sockaddr(addr, sa);

  EXPECT_EQ(sa.sun_family, AF_UNIX);
  EXPECT_EQ(alen, static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + addr.size()));
  EXPECT_EQ(std::memcmp(sa.sun_path, addr.data(), addr.size()), 0);
  // Bytes past the address must remain zero (i.e. no trailing NUL is required
  // and no leftover poison bits leak to the kernel).
  for (size_t i = addr.size(); i < sizeof(sa.sun_path); ++i) {
    EXPECT_EQ(static_cast<unsigned char>(sa.sun_path[i]), 0u) << "byte " << i;
  }
}

TEST(BridgeUdsAddrTest, MaxLengthAddressAccepted)
{
  sockaddr_un probe{};
  // Boundary: exactly sizeof(sun_path) bytes (leading NUL + sizeof-1 name bytes).
  std::string boundary(sizeof(probe.sun_path), 'y');
  boundary[0] = '\0';

  sockaddr_un sa{};
  const socklen_t alen = agnocast::detail::fill_abstract_sockaddr(boundary, sa);
  EXPECT_EQ(alen, static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + boundary.size()));
  EXPECT_EQ(std::memcmp(sa.sun_path, boundary.data(), boundary.size()), 0);
}

// ---- create_bridge_uds_listener -------------------------------------------

TEST(BridgeUdsListenerTest, RejectsInvalidAddress)
{
  EXPECT_THROW(agnocast::create_bridge_uds_listener(""), std::invalid_argument);
  EXPECT_THROW(agnocast::create_bridge_uds_listener("no-leading-nul"), std::invalid_argument);
}

TEST(BridgeUdsListenerTest, DuplicateBindFailsWithAddrInUse)
{
  const std::string addr = make_unique_abstract_addr("dup");
  ScopedFd first(agnocast::create_bridge_uds_listener(addr));
  ASSERT_NE(first.get(), -1);

  try {
    ScopedFd second(agnocast::create_bridge_uds_listener(addr));
    FAIL() << "second listener on the same abstract address must fail";
  } catch (const std::system_error & e) {
    EXPECT_EQ(e.code().value(), EADDRINUSE)
      << "expected EADDRINUSE, got " << e.code().value() << " (" << e.what() << ")";
  }
}

// ---- send_bridge_uds_message ----------------------------------------------

TEST(BridgeUdsSendTest, RoundTripDeliversPayload)
{
  const std::string addr = make_unique_abstract_addr("roundtrip");
  ScopedFd listener(agnocast::create_bridge_uds_listener(addr));
  ASSERT_NE(listener.get(), -1);

  const std::array<std::uint8_t, 4> payload = {0xDE, 0xAD, 0xBE, 0xEF};
  const auto logger = rclcpp::get_logger("BridgeUdsSendTest");
  EXPECT_TRUE(agnocast::send_bridge_uds_message(addr, payload.data(), payload.size(), logger));

  std::array<std::uint8_t, 32> buf{};
  const ssize_t n = recv(listener.get(), buf.data(), buf.size(), 0);
  ASSERT_EQ(n, static_cast<ssize_t>(payload.size()))
    << "recv failed or short read: " << (n < 0 ? strerror(errno) : "size mismatch");
  EXPECT_EQ(std::memcmp(buf.data(), payload.data(), payload.size()), 0);
}

TEST(BridgeUdsSendTest, InvalidAddressFailsFast)
{
  const auto logger = rclcpp::get_logger("BridgeUdsSendTest");
  const std::uint8_t byte = 0;
  // Empty and non-NUL-prefixed addresses are rejected inside
  // send_bridge_uds_message before any sendto()/retry loop runs, so this must
  // return false immediately rather than spending
  // BRIDGE_UDS_SEND_MAX_RETRIES * BRIDGE_UDS_SEND_RETRY_INTERVAL_US retrying.
  EXPECT_FALSE(agnocast::send_bridge_uds_message("", &byte, sizeof(byte), logger));
  EXPECT_FALSE(agnocast::send_bridge_uds_message("no-nul", &byte, sizeof(byte), logger));
}
