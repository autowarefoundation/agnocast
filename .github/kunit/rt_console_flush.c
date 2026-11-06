// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
//
// Built only into the PREEMPT_RT KUnit kernel of .github/workflows/kunit.yaml. PREEMPT_RT prints
// the serial console from a kthread, and v6.12 restarts without flushing it, so
// kunit_shutdown=reboot can drop the last KTAP lines and kunit.py reports a crash.
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

static int wait_for_console(struct notifier_block * nb, unsigned long action, void * data)
{
  // A fixed delay, because v6.12 keeps pr_flush() static. If the console needs more than
  // 1 s to drain, the last KTAP lines can still get lost.
  msleep(1000);
  return NOTIFY_DONE;
}

static struct notifier_block wait_for_console_nb = {.notifier_call = wait_for_console};

static int __init wait_for_console_init(void)
{
  return register_reboot_notifier(&wait_for_console_nb);
}
late_initcall(wait_for_console_init);
