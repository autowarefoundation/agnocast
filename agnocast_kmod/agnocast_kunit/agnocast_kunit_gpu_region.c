// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_kunit_gpu_region.h"

#include "../agnocast.h"

#include <kunit/test.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>

static const char * TOPIC_NAME = "/kunit_test_topic";
static const char * NODE_NAME = "/kunit_test_node";
static const pid_t PUBLISHER_PID = 1000;
static const pid_t SUBSCRIBER_PID = 1001;
static const uint32_t QOS_DEPTH = 10;
// A macro rather than a static: checkpatch rejects a static initialised to
// false, and CI lints this PR's diff rather than the tree, so the sibling
// suites' `static const bool` is not a precedent that passes.
#define IS_BRIDGE false

static const uint32_t SLOT_SIZE = 2048;
static const uint32_t SLOT_COUNT = 4;
static const uint64_t MAPPED_SIZE = 8192;

// A stand-in for a GPU memory handle. The module never interprets the file, only
// holds a reference on it, so any file exercises the reference handling. The
// descriptor install is out of reach here: it depends on the calling process's
// file table (see gpu_region_e2e.cpp).
//
// An anonymous inode rather than filp_open("/dev/null"): these tests also run
// under kunit.py, which boots a kernel with no root filesystem mounted, so
// opening a path would fail and abort most of the suite.
static const struct file_operations handle_file_fops = {
  .owner = THIS_MODULE,
};

static struct file * open_handle_file(struct kunit * test)
{
  struct file * file =
    anon_inode_getfile("agnocast_kunit_gpu_handle", &handle_file_fops, NULL, O_RDONLY);
  KUNIT_ASSERT_FALSE(test, IS_ERR(file));
  return file;
}

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

// GET is authorized against a subscriber of the topic, so every test that reads
// a region back needs one.
static topic_local_id_t setup_subscriber(struct kunit * test)
{
  union ioctl_add_subscriber_args add_sub_args;
  int ret = agnocast_ioctl_add_subscriber(
    TOPIC_NAME, current->nsproxy->ipc_ns, NODE_NAME, SUBSCRIBER_PID, QOS_DEPTH, false, true, false,
    false, IS_BRIDGE, &add_sub_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  return add_sub_args.ret_id;
}

static void fill_args(
  union ioctl_add_gpu_region_args * args, const topic_local_id_t publisher_id,
  const uint32_t slot_size, const uint32_t slot_count, const uint64_t mapped_size)
{
  memset(args, 0, sizeof(*args));
  args->publisher_id = publisher_id;
  args->backend_type = AGNOCAST_GPU_BACKEND_VMM;
  args->slot_size = slot_size;
  args->slot_count = slot_count;
  args->mapped_size = mapped_size;
  for (int i = 0; i < GPU_DEVICE_UUID_SIZE; i++) {
    args->device_uuid[i] = (uint8_t)(0xA0 + i);
  }
  args->handle_fd = -1;  // the core takes the file directly, not a descriptor
}

// Registers a VMM region with a real file reference, as the ioctl wrapper would
// after resolving the caller's descriptor.
static uint32_t add_region(
  struct kunit * test, const topic_local_id_t publisher_id, const uint32_t slot_size,
  const uint32_t slot_count, const uint64_t mapped_size)
{
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file = open_handle_file(test);
  int ret;

  fill_args(&add_args, publisher_id, slot_size, slot_count, mapped_size);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  if (ret != 0) fput(handle_file);  // not consumed on failure
  KUNIT_ASSERT_EQ(test, ret, 0);

  return add_args.ret_region_id;
}

void test_case_gpu_region_round_trip(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  const uint32_t assigned_region_id =
    add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);

  // region_id 0 asks for "any", which is what a caller that has not yet seen a
  // message uses.
  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, 0, NULL, 0,
    &get_args, &handle_file);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_backend_type, (uint32_t)AGNOCAST_GPU_BACKEND_VMM);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_size, SLOT_SIZE);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_count, SLOT_COUNT);
  KUNIT_EXPECT_EQ(test, get_args.ret_mapped_size, MAPPED_SIZE);
  KUNIT_EXPECT_EQ(test, get_args.ret_blob_size, 0u);
  // The id a message would carry must be the same on both sides of the module.
  KUNIT_EXPECT_NE(test, get_args.ret_region_id, 0u);
  KUNIT_EXPECT_EQ(test, get_args.ret_region_id, assigned_region_id);
  // A reference for the caller to install, distinct from the module's own.
  KUNIT_EXPECT_PTR_NE(test, handle_file, NULL);
  // The descriptor number is the wrapper's to fill in; the core reports none.
  KUNIT_EXPECT_EQ(test, get_args.ret_handle_fd, -1);
  for (int i = 0; i < GPU_DEVICE_UUID_SIZE; i++) {
    KUNIT_EXPECT_EQ(test, get_args.ret_device_uuid[i], (uint8_t)(0xA0 + i));
  }

  if (handle_file) fput(handle_file);
}

void test_case_gpu_region_blob(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  uint8_t blob[8];
  uint8_t out[8];
  int ret;

  for (int i = 0; i < 8; i++) blob[i] = (uint8_t)(i + 1);

  // A descriptor blob is the NvSciBuf shape: no file, bytes instead.
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.backend_type = AGNOCAST_GPU_BACKEND_NVSCIBUF;
  add_args.blob_size = sizeof(blob);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, NULL, blob);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&get_args, 0, sizeof(get_args));
  memset(out, 0, sizeof(out));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, 0, out,
    sizeof(out), &get_args, &handle_file);

  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_blob_size, (uint32_t)sizeof(blob));
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
  for (int i = 0; i < 8; i++) {
    KUNIT_EXPECT_EQ(test, out[i], (uint8_t)(i + 1));
  }
}

// The handle must be the kind the declared mechanism uses; a mismatch would only
// surface in the importer.
void test_case_gpu_region_reject_handle_mechanism_mismatch(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file;
  uint8_t blob[8] = {0};
  int ret;

  // VMM without a file.
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, NULL, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  // VMM with a file and a blob, which names two mechanisms at once.
  handle_file = open_handle_file(test);
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.blob_size = sizeof(blob);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, blob);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  fput(handle_file);

  // NvSciBuf with a file instead of a descriptor.
  handle_file = open_handle_file(test);
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.backend_type = AGNOCAST_GPU_BACKEND_NVSCIBUF;
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  fput(handle_file);

  // A mechanism the module has never heard of.
  handle_file = open_handle_file(test);
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.backend_type = 99;
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  fput(handle_file);
}

// Geometry arrives from another process: slots that do not fit their mapping
// must be refused rather than become out-of-bounds addresses in the importer.
void test_case_gpu_region_reject_geometry_over_mapping(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file = open_handle_file(test);
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE - 1);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  fput(handle_file);
}

void test_case_gpu_region_reject_empty_geometry(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file = open_handle_file(test);
  int ret;

  fill_args(&add_args, publisher_id, 0, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  fill_args(&add_args, publisher_id, SLOT_SIZE, 0, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  fput(handle_file);
}

// A publisher grows by adding regions, so a second registration is normal and
// must yield a distinct id: each message records the id of the region it used.
void test_case_gpu_region_second_registration_gets_a_new_id(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);

  const uint32_t first_id = add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  const uint32_t second_id =
    add_region(test, publisher_id, SLOT_SIZE * 2, SLOT_COUNT, MAPPED_SIZE * 2);

  KUNIT_EXPECT_NE(test, first_id, second_id);
}

// The id must select that region and not merely any of the publisher's, or a
// slot index would be applied to the wrong geometry.
void test_case_gpu_region_get_selects_the_named_region(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  const uint32_t first_id = add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  const uint32_t second_id =
    add_region(test, publisher_id, SLOT_SIZE * 2, SLOT_COUNT, MAPPED_SIZE * 2);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, second_id,
    NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_region_id, second_id);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_size, SLOT_SIZE * 2);
  if (handle_file) fput(handle_file);

  handle_file = NULL;
  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, first_id,
    NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_EQ(test, get_args.ret_region_id, first_id);
  KUNIT_EXPECT_EQ(test, get_args.ret_slot_size, SLOT_SIZE);
  if (handle_file) fput(handle_file);
}

// An id this publisher does not own must resolve to nothing rather than to
// whichever region happens to be first.
void test_case_gpu_region_get_unknown_id(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  const uint32_t region_id = add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id,
    region_id + 1000, NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

void test_case_gpu_region_reject_beyond_the_cap(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file;
  int ret;

  for (int i = 0; i < MAX_GPU_REGION_NUM_PER_PUBLISHER; i++) {
    add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  }

  handle_file = open_handle_file(test);
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -ENOSPC);
  fput(handle_file);
}

// Removal is what keeps the cap from being terminal: an emptied region can be
// traded for a differently sized one.
void test_case_gpu_region_remove_frees_a_slot_under_the_cap(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file;
  uint32_t first_id = 0;
  int ret;

  for (int i = 0; i < MAX_GPU_REGION_NUM_PER_PUBLISHER; i++) {
    const uint32_t id = add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
    if (i == 0) first_id = id;
  }

  ret = agnocast_ioctl_remove_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, publisher_id, first_id);
  KUNIT_EXPECT_EQ(test, ret, 0);

  // The freed slot is usable again.
  handle_file = open_handle_file(test);
  fill_args(&add_args, publisher_id, SLOT_SIZE * 2, SLOT_COUNT, MAPPED_SIZE * 2);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  if (ret != 0) fput(handle_file);
  KUNIT_EXPECT_EQ(test, ret, 0);
  KUNIT_EXPECT_NE(test, add_args.ret_region_id, first_id);
}

// A removed region must be gone for importers too, or a subscriber would map
// memory the publisher has released.
void test_case_gpu_region_remove_makes_it_unreachable(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  const uint32_t region_id = add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);

  ret = agnocast_ioctl_remove_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, publisher_id, region_id);
  KUNIT_EXPECT_EQ(test, ret, 0);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, region_id,
    NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);

  // Removing it twice must not double-free the reference it held.
  ret = agnocast_ioctl_remove_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, publisher_id, region_id);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
}

void test_case_gpu_region_remove_rejects_a_foreign_process(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  int ret;

  const uint32_t region_id = add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);

  ret = agnocast_ioctl_remove_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID + 1, publisher_id, region_id);
  KUNIT_EXPECT_EQ(test, ret, -EPERM);

  // "Any" would let a caller release a region it cannot name.
  ret = agnocast_ioctl_remove_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, publisher_id, 0);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

void test_case_gpu_region_add_topic_not_found(struct kunit * test)
{
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file = open_handle_file(test);
  int ret;

  fill_args(&add_args, 0, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    "/kunit_absent_topic", current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  fput(handle_file);
}

void test_case_gpu_region_add_publisher_not_found(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file = open_handle_file(test);
  int ret;

  fill_args(&add_args, publisher_id + 1, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);

  fput(handle_file);
}

// Only the owning process may register memory under a publisher's name.
void test_case_gpu_region_add_rejects_a_foreign_process(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  union ioctl_add_gpu_region_args add_args;
  struct file * handle_file = open_handle_file(test);
  int ret;

  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID + 1, &add_args, handle_file, NULL);
  KUNIT_EXPECT_EQ(test, ret, -EPERM);

  fput(handle_file);
}

void test_case_gpu_region_get_publisher_not_found(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id + 1, subscriber_id, 0, NULL,
    0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -EINVAL);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

// Knowing a topic name and a publisher id must not be enough to be handed a
// descriptor; see the trust boundary in docs/gpu_ipc.md.
void test_case_gpu_region_get_rejects_an_unauthorized_caller(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  add_region(test, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);

  // A process that is not the subscriber it names.
  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID + 1, publisher_id, subscriber_id, 0, NULL,
    0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -EPERM);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);

  // A subscriber id that belongs to no subscription on this topic.
  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id + 1000, 0,
    NULL, 0, &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -EPERM);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

void test_case_gpu_region_get_without_registration(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  int ret;

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, 0, NULL, 0,
    &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOENT);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}

void test_case_gpu_region_get_blob_buffer_too_small(struct kunit * test)
{
  const topic_local_id_t publisher_id = setup_publisher(test);
  const topic_local_id_t subscriber_id = setup_subscriber(test);
  union ioctl_add_gpu_region_args add_args;
  union ioctl_get_gpu_region_args get_args;
  struct file * handle_file = NULL;
  uint8_t blob[8];
  uint8_t out[4];
  int ret;

  memset(blob, 0x5A, sizeof(blob));
  fill_args(&add_args, publisher_id, SLOT_SIZE, SLOT_COUNT, MAPPED_SIZE);
  add_args.backend_type = AGNOCAST_GPU_BACKEND_NVSCIBUF;
  add_args.blob_size = sizeof(blob);
  ret = agnocast_ioctl_add_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, PUBLISHER_PID, &add_args, NULL, blob);
  KUNIT_ASSERT_EQ(test, ret, 0);

  memset(&get_args, 0, sizeof(get_args));
  ret = agnocast_ioctl_get_gpu_region(
    TOPIC_NAME, current->nsproxy->ipc_ns, SUBSCRIBER_PID, publisher_id, subscriber_id, 0, out,
    sizeof(out), &get_args, &handle_file);
  KUNIT_EXPECT_EQ(test, ret, -ENOSPC);
  KUNIT_EXPECT_PTR_EQ(test, handle_file, NULL);
}
