// SPDX-License-Identifier: GPL-2.0-only OR BSD-2-Clause
#include "agnocast_internal.h"

// =========================================
// Initialize and cleanup

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 2, 0)
static char * agnocast_devnode(const struct device * dev, umode_t * mode)
#else
static char * agnocast_devnode(struct device * dev, umode_t * mode)
#endif
{
  if (mode) {
    *mode = 0666;
  }
  return NULL;
}

static struct file_operations fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = agnocast_ioctl,
};

static int exit_worker_thread(void * data)
{
  while (!kthread_should_stop()) {
    wait_event_interruptible(worker_wait, smp_load_acquire(&has_new_pid) || kthread_should_stop());

    if (kthread_should_stop()) break;

    // Drain all queued PIDs in a single wake-up cycle
    while (true) {
      pid_t pid;
      unsigned long flags;
      bool got_pid = false;

      spin_lock_irqsave(&pid_queue_lock, flags);

      if (queue_head != queue_tail) {
        pid = exit_pid_queue[queue_head];
        queue_head = (queue_head + 1) & EXIT_QUEUE_MASK;
        got_pid = true;
      }

      if (queue_head == queue_tail) smp_store_release(&has_new_pid, 0);

      spin_unlock_irqrestore(&pid_queue_lock, flags);

      if (!got_pid) break;

      agnocast_process_exit_cleanup(pid);
    }
  }

  return 0;
}

static void find_sched_process_exit_tp(struct tracepoint * tp, void * priv)
{
  if (strcmp(tp->name, "sched_process_exit") == 0) {
    tp_sched_process_exit = tp;
  }
}

int agnocast_init_device(void)
{
  int ret;

  major = register_chrdev(0, "agnocast" /*device driver name*/, &fops);
  if (major < 0) {
    pr_err("agnocast: register_chrdev failed: %d\n", major);
    return major;
  }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
  agnocast_class = class_create("agnocast_class");
#else
  agnocast_class = class_create(THIS_MODULE, "agnocast_class");
#endif
  if (IS_ERR(agnocast_class)) {
    ret = PTR_ERR(agnocast_class);
    pr_err("agnocast: class_create failed: %d\n", ret);
    goto err_unregister_chrdev;
  }

  agnocast_class->devnode = agnocast_devnode;
  agnocast_device =
    device_create(agnocast_class, NULL, MKDEV(major, 0), NULL, "agnocast" /*file name*/);
  if (IS_ERR(agnocast_device)) {
    ret = PTR_ERR(agnocast_device);
    pr_err("agnocast: device_create failed: %d\n", ret);
    goto err_class_destroy;
  }

  return 0;

err_class_destroy:
  class_destroy(agnocast_class);
err_unregister_chrdev:
  unregister_chrdev(major, "agnocast");
  return ret;
}

int agnocast_init_kthread(void)
{
  queue_head = queue_tail = 0;

  worker_task = kthread_run(exit_worker_thread, NULL, "agnocast_exit_worker");
  if (IS_ERR(worker_task)) {
    dev_warn(agnocast_device, "failed to create kernel thread. (%s)\n", __func__);
    return PTR_ERR(worker_task);
  }

  return 0;
}

int agnocast_init_exit_hook(void)
{
  int ret;

  for_each_kernel_tracepoint(find_sched_process_exit_tp, NULL);
  if (!tp_sched_process_exit) {
    dev_warn(agnocast_device, "sched_process_exit tracepoint not found\n");
    return -ENOENT;
  }

  ret = tracepoint_probe_register(tp_sched_process_exit, agnocast_process_exit, NULL);
  if (ret) {
    dev_warn(agnocast_device, "tracepoint_probe_register failed: %d\n", ret);
    return ret;
  }

  return 0;
}
