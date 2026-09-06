// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_gpu_region.h"

#include "../agnocast.h"

#include <kunit/test.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const pid_t PUBLISHER_PID = 1000;
static const uint32_t QOS_DEPTH = 10;
static const bool IS_BRIDGE = false;

static const uint32_t SLOT_SIZE = 2048;
static const uint32_t SLOT_COUNT = 4;
static const uint64_t MAPPED_SIZE = 8192;

// The file reference is resolved in the ioctl wrapper, not in the core, so the
// cores are exercised here with none. That mirrors a mechanism whose handle is
// not a file descriptor, and leaves the fd install untested by construction:
// it depends on the calling process's file table, which KUnit cannot provide.
static topic_local_id_t setup_publisher(struct kunit * test)
{
  union ioctl_add_process_args add_process_args;
  union ioctl_add_publisher_args add_pub_args;
  int ret;

  ret = agnocast_ioctl_add_process(
    PUBLISHER_PID, current->nsproxy->ipc_ns, false, 0, &add_process_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  ret = agnocast_ioctl_add_publisher(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, PUBLISHER_PID, QOS_DEPTH, false, IS_BRIDGE,
    &add_pub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);

  return add_pub_args.ret_id;
}

static void fill_args(
  union ioctl_add_gpu_region_args * args, const topic_local_id_t publisher_id,
  const uint32_t slot_size, const uint32_t slot_count, const uint64_t mapped_size)
{
  memset(args, 0, sizeof(*args));
  args->publisher_id = publisher_id;
  args->backend_type = 1;  // Vmm
  args->slot_size = slot_size;
  args->slot_count = slot_count;
  args->mapped_size = mapped_size;
  for (int i = 0; i < GPU_DEVICE_UUID_SIZE; i++) {
    args->device_uuid[i] = (uint8_t)(0xA0 + i);
  }
  args->handle_fd = -1;
}

void test_case_gpu_region_round_trip(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_ASSERT_EQ(test, ret, 0);
  const uint32_t assigned_region_id = add_args.ret_region_id;

  // region_id 0 asks for "any", which is what a caller that has not yet seen a
  // message uses.
  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, 0, NULL, 0, &get_args, &handle_file);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_backend_type, 1u);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_size, SLOT_SIZE);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_count, SLOT_COUNT);
  KUNIT_EXPECT_EQ(test, get_args.ret_mapped_size, MAPPED_SIZE);
  KUNIT_EXPECT_EQ(test, get_args.ret_blob_size, 0u);
  // The id a message would carry must be the same on both sides of the module.
  KUNIT_EXPECT_NE(test, get_args.ret_region_id, 0u);
  KUNIT_EXPECT_EQ(test, get_args.ret_region_id, assigned_region_id);
  // No descriptor was registered, so none is handed back for installation.
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, get_args.ret_handle_fd, -1);
  for (int i = 0; i < GPU_DEVICE_UUID_SIZE; i++) {
    KUNIT_EXPECT_EQ(test, get_args.ret_device_uuid[i], (uint8_t)(0xA0 + i));
  }
}

void test_case_gpu_region_blob(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  uint8_t blob[8];
  uint8_t out[8];
  int ret;

  for (int i = 0; i < 8; i++) blob[i] = (uint8_t)(i + 1);

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.blob_size = sizeof(blob);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, blob);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&get_args, 0, sizeof(get_args));
  memset(out, 0, sizeof(out));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, 0, out, sizeof(out), &get_args,
    &handle_file);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_blob_size, (uint32_t)sizeof(blob));
  for (int i = 0; i < 8; i++) {
    KUNIT_EXPECT_EQ(test, out[i], (uint8_t)(i + 1));
  }
}

// Geometry arrives from another process, so a region whose slots do not fit its
// mapping must be refused here rather than trusted and turned into out-of-bounds
// device addresses by the importer.
void test_case_gpu_region_reject_geometry_over_mapping(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE - 1);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_gpu_region_reject_empty_geometry(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  int ret;

  fill_args(&add_args, publisher_id, 0, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  fill_args(&add_args, publisher_id, SLOT_SIZE, 0, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

// A publisher grows its pool by adding regions, so a second registration is
// normal and must yield a distinct id: a message names the region it used.
void test_case_gpu_region_second_registration_gets_a_new_id(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_ASSERT_EQ(test, ret, 0);
  const uint32_t first_id = add_args.ret_region_id;

  fill_args(&add_args, publisher_id, SLOT_SIZE * 2, SLOT_COUNT, MAPPED_SIZE * 2);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_ASSERT_EQ(test, ret, 0);

  KUNIT_EXPECT_NE(test, first_id, add_args.ret_region_id);
}

// The id a message carries must select that region and not merely any of the
// publisher's, otherwise a slot index would be applied to the wrong geometry.
void test_case_gpu_region_get_selects_the_named_region(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_ASSERT_EQ(test, ret, 0);
  const uint32_t first_id = add_args.ret_region_id;

  fill_args(&add_args, publisher_id, SLOT_SIZE * 2, SLOT_COUNT, MAPPED_SIZE * 2);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_ASSERT_EQ(test, ret, 0);
  const uint32_t second_id = add_args.ret_region_id;

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, second_id, NULL, 0, &get_args,
    &handle_file);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_region_id, second_id);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_size, SLOT_SIZE * 2);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, first_id, NULL, 0, &get_args,
    &handle_file);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_region_id, first_id);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_size, SLOT_SIZE);
}

// Ids are never reused, so one that names a region this publisher does not own
// must resolve to nothing rather than to whichever region happens to be first.
void test_case_gpu_region_get_unknown_id(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, add_args.ret_region_id + 1000, NULL, 0,
    &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

// Growth is driven by a userspace process, and each region pins device memory
// plus a file reference the module holds until the publisher is gone.
void test_case_gpu_region_reject_beyond_the_cap(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  int ret;

  for (int i = 0; i < MAX_GPU_REGION_NUM_PER_PUBLISHER; i++) {
    fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
    ret =
      agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
    KUNIT_ASSERT_EQ(test, ret, 0);
  }

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -ENOSPC);
}

void test_case_gpu_region_add_topic_not_found(struct kunit * test)
{
  union ioctl_add_gpu_region_args add_args;
  int ret;

  fill_args(&add_args, 0, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    "/kunit_absent_topic", current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_gpu_region_add_publisher_not_found(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  int ret;

  fill_args(&add_args, publisher_id + 1, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_gpu_region_get_publisher_not_found(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id + 1, 0, 0, NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

void test_case_gpu_region_get_without_registration(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, 0, NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

void test_case_gpu_region_get_blob_buffer_too_small(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  uint8_t blob[8];
  uint8_t out[4];
  int ret;

  memset(blob, 0x5A, sizeof(blob));
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.blob_size = sizeof(blob);
  ret = agnocast_ioctl_add_gpu_region(TOPIC_NAME, current->nsproxy->ipc_ns, &add_args, NULL, blob);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, publisher_id, 0, 0, out, sizeof(out), &get_args,
    &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOSPC);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}
