#include "agnocast/agnocast_callback_info.hpp"
#include "agnocast/agnocast_epoll_event.hpp"

#include <gtest/gtest.h>

#include <array>

namespace agnocast
{

TEST(EpollEventDataTest, PackAndUnpackRoundTripForAllKnownTypes)
{
  constexpr EpollEventLocalID id = 0x12345678;

  const std::array<EpollEventType, static_cast<size_t>(EpollEventType::NrEventType)> types = {
    EpollEventType::Subscription, EpollEventType::Timer, EpollEventType::Clock,
    EpollEventType::Shutdown};
  static_assert(
    types.size() == static_cast<size_t>(EpollEventType::NrEventType),
    "Update this test when adding a new EpollEventType");

  for (auto type : types) {
    const auto packed = pack_epoll_data(type, id);
    const auto [unpacked_type, unpacked_id] = unpack_epoll_data(packed);

    EXPECT_EQ(unpacked_type, type);
    EXPECT_EQ(unpacked_id, id);
  }
}

TEST(EpollEventDataTest, PackPlacesTypeInUpperBitsAndIdInLowerBits)
{
  constexpr auto type = EpollEventType::Shutdown;
  constexpr EpollEventLocalID id = 0x89ABCDEF;

  const auto packed = pack_epoll_data(type, id);

  EXPECT_EQ(packed >> EPOLL_DATA_TYPE_SHIFT, static_cast<uint64_t>(type));
  EXPECT_EQ(packed & static_cast<uint64_t>(EPOLL_DATA_ID_BITMASK), static_cast<uint64_t>(id));
}

TEST(EpollEventDataTest, PackAndUnpackWithMaxID)
{
  constexpr auto type = EpollEventType::Subscription;
  constexpr EpollEventLocalID id = MAX_CALLBACK_INFO_ID - 1;

  const auto packed = pack_epoll_data(type, id);
  const auto [unpacked_type, unpacked_id] = unpack_epoll_data(packed);

  EXPECT_EQ(unpacked_type, type);
  EXPECT_EQ(unpacked_id, id);
}

}  // namespace agnocast
