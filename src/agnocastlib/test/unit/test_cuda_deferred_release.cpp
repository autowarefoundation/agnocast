// Unit tests for the GPU-IPC "done edge": reader-local deferred release.
//
// No GPU is involved. The three read-done primitives that normally live in
// libagnocast_cuda are installed as test doubles, and agnocast::release_subscriber_
// reference() is overridden here (the executable's definition interposes the one in
// libagnocast.so, the same technique test_mocked_agnocast.cpp uses) so the test can
// observe exactly when the kernel-side reference would be released.
#include "agnocast/agnocast_smart_pointer.hpp"
#include "agnocast/cuda_deferred_release.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace
{

// Recorded kernel-side releases.
struct ReleasedReference
{
  std::string topic_name;
  agnocast::topic_local_id_t pubsub_id;
  int64_t entry_id;
};
std::vector<ReleasedReference> g_released;

// Test double for libagnocast_cuda's read-done markers. A marker is an index into
// `g_completed`, offset by one so no token is ever null.
struct FakeMarkers
{
  std::vector<bool> completed;
  std::vector<int> record_stream_kind;
  std::vector<void *> record_stream;
  int record_failures_remaining = 0;
  size_t record_calls = 0;
  size_t query_calls = 0;
  size_t wait_calls = 0;
};
FakeMarkers g_markers;

void * token_from_index(size_t index)
{
  return reinterpret_cast<void *>(static_cast<std::uintptr_t>(index + 1));
}

size_t index_from_token(void * token)
{
  return static_cast<size_t>(reinterpret_cast<std::uintptr_t>(token)) - 1;
}

int fake_record(int stream_kind, void * stream, void ** out_token)
{
  ++g_markers.record_calls;
  if (g_markers.record_failures_remaining > 0) {
    --g_markers.record_failures_remaining;
    return 0;
  }
  g_markers.completed.push_back(false);
  g_markers.record_stream_kind.push_back(stream_kind);
  g_markers.record_stream.push_back(stream);
  *out_token = token_from_index(g_markers.completed.size() - 1);
  return 1;
}

int fake_query(void * token)
{
  ++g_markers.query_calls;
  return g_markers.completed[index_from_token(token)] ? 1 : 0;
}

void fake_wait(void * token)
{
  ++g_markers.wait_calls;
  g_markers.completed[index_from_token(token)] = true;
}

void complete_marker(size_t index)
{
  g_markers.completed[index] = true;
}

class CudaDeferredRelease : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Flush first: it clears any entry left by a previous test AND uninstalls the hooks,
    // so the install below always takes effect (set_gpu_done_ops is first-wins).
    agnocast::flush_deferred_subscriber_releases();
    g_released.clear();
    g_markers = FakeMarkers{};
    agnocast::set_gpu_done_ops(agnocast::GpuDoneOps{&fake_record, &fake_query, &fake_wait});
  }

  void TearDown() override
  {
    agnocast::flush_deferred_subscriber_releases();
    g_released.clear();
    g_markers = FakeMarkers{};
  }

  static bool defer(const std::string & topic, int64_t entry_id, void * stream = nullptr)
  {
    return agnocast::defer_subscriber_release(
      topic, 7, entry_id, stream == nullptr ? 0 : 2, stream);
  }
};

}  // namespace

namespace agnocast
{
// Interposes libagnocast.so's definition for this test binary.
void release_subscriber_reference(
  const std::string & topic_name, const topic_local_id_t pubsub_id, const int64_t entry_id)
{
  g_released.push_back(ReleasedReference{topic_name, pubsub_id, entry_id});
}
}  // namespace agnocast

TEST_F(CudaDeferredRelease, ReleaseIsHeldUntilTheMarkerCompletes)
{
  ASSERT_TRUE(defer("/cuda_pointcloud", 42));
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 1u);
  EXPECT_TRUE(g_released.empty());

  // Draining while the reader's GPU work is still in flight must not release: that is
  // exactly the write-after-read corruption the done edge exists to prevent.
  agnocast::drain_deferred_subscriber_releases();
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 1u);
  EXPECT_TRUE(g_released.empty());

  complete_marker(0);
  agnocast::drain_deferred_subscriber_releases();
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 0u);
  ASSERT_EQ(g_released.size(), 1u);
  EXPECT_EQ(g_released[0].topic_name, "/cuda_pointcloud");
  EXPECT_EQ(g_released[0].pubsub_id, 7);
  EXPECT_EQ(g_released[0].entry_id, 42);
}

TEST_F(CudaDeferredRelease, TheDeclaredStreamIsForwardedToTheMarker)
{
  auto * stream = reinterpret_cast<void *>(static_cast<std::uintptr_t>(0xabc000));
  ASSERT_TRUE(defer("/topic", 1, stream));
  ASSERT_EQ(g_markers.record_stream.size(), 1u);
  EXPECT_EQ(g_markers.record_stream[0], stream);
  EXPECT_EQ(g_markers.record_stream_kind[0], 2);  // CudaStreamKind::kExplicit
}

TEST_F(CudaDeferredRelease, CompletionIsNotFifoAcrossStreams)
{
  // Different subscriptions record on different streams, so a later marker can finish
  // first; the drain must not stop at the first pending entry.
  ASSERT_TRUE(defer("/a", 1));
  ASSERT_TRUE(defer("/b", 2));
  ASSERT_TRUE(defer("/c", 3));

  complete_marker(2);
  agnocast::drain_deferred_subscriber_releases();
  ASSERT_EQ(g_released.size(), 1u);
  EXPECT_EQ(g_released[0].entry_id, 3);
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 2u);

  complete_marker(0);
  complete_marker(1);
  agnocast::drain_deferred_subscriber_releases();
  EXPECT_EQ(g_released.size(), 3u);
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 0u);
}

TEST_F(CudaDeferredRelease, ReturnsFalseWhenTheMarkerCannotBeRecorded)
{
  g_markers.record_failures_remaining = 1;
  // The caller (ipc_shared_ptr::reset) then releases immediately rather than leaking
  // the reference.
  EXPECT_FALSE(defer("/topic", 1));
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 0u);
  EXPECT_TRUE(g_released.empty());
}

TEST_F(CudaDeferredRelease, DrainIsCheapAndSafeWhenNothingIsDeferred)
{
  agnocast::drain_deferred_subscriber_releases();
  EXPECT_EQ(g_markers.query_calls, 0u);
  EXPECT_TRUE(g_released.empty());
}

TEST_F(CudaDeferredRelease, AFullRingBlocksOnTheOldestInsteadOfReleasingEarly)
{
  for (size_t i = 0; i < agnocast::MAX_DEFERRED_SUBSCRIBER_RELEASES; ++i) {
    ASSERT_TRUE(defer("/topic", static_cast<int64_t>(i)));
  }
  EXPECT_EQ(
    agnocast::deferred_subscriber_release_count(), agnocast::MAX_DEFERRED_SUBSCRIBER_RELEASES);
  EXPECT_TRUE(g_released.empty());

  // One more deferral must not drop a reference whose reads may still be running; it
  // waits for the oldest marker instead. That is a slowdown, never corruption.
  ASSERT_TRUE(defer("/topic", 999));
  EXPECT_EQ(g_markers.wait_calls, 1u);
  ASSERT_EQ(g_released.size(), 1u);
  EXPECT_EQ(g_released[0].entry_id, 0);
  EXPECT_EQ(
    agnocast::deferred_subscriber_release_count(), agnocast::MAX_DEFERRED_SUBSCRIBER_RELEASES);
}

TEST_F(CudaDeferredRelease, FlushReleasesEverythingWithoutTouchingTheMarkers)
{
  ASSERT_TRUE(defer("/a", 1));
  ASSERT_TRUE(defer("/b", 2));
  const size_t queries_before = g_markers.query_calls;

  // Flush runs from static destruction, after libagnocast_cuda's proxy singleton has
  // already been destroyed, so it must not query or wait on a marker — and it must not be
  // able to hang on a stuck reader.
  agnocast::flush_deferred_subscriber_releases();
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 0u);
  EXPECT_EQ(g_released.size(), 2u);
  EXPECT_EQ(g_markers.query_calls, queries_before);
  EXPECT_EQ(g_markers.wait_calls, 0u);
}

TEST_F(CudaDeferredRelease, DeferralIsOffAfterAFlush)
{
  agnocast::flush_deferred_subscriber_releases();
  // Teardown has begun: a later release must go straight through (the caller releases
  // immediately) rather than call into the already-destroyed proxy.
  EXPECT_FALSE(defer("/a", 1));
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 0u);
}

TEST_F(CudaDeferredRelease, DeferralOpportunisticallyRetiresFinishedEntries)
{
  ASSERT_TRUE(defer("/a", 1));
  complete_marker(0);

  // The next deferral drains first, so a steady stream of messages keeps the ring
  // shallow without depending on the executor's drain point.
  ASSERT_TRUE(defer("/b", 2));
  ASSERT_EQ(g_released.size(), 1u);
  EXPECT_EQ(g_released[0].entry_id, 1);
  EXPECT_EQ(agnocast::deferred_subscriber_release_count(), 1u);
}
