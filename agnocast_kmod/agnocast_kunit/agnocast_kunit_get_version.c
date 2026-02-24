#include "agnocast_kunit_get_version.h"

#include "../agnocast.h"

#include <kunit/test.h>

void test_case_get_version(struct kunit * test)
{
  struct ioctl_get_version_args version_args;
  int ret;

  ret = agnocast_ioctl_get_version(&version_args);
  KUNIT_ASSERT_EQ(test, ret, 0);
  KUNIT_EXPECT_NE(test, version_args.ret_version[0], '\0');
}
