KUNIT_TEST_OBJS := \
  agnocast_kunit_add_process.o \
  agnocast_kunit_add_subscriber.o \
  agnocast_kunit_add_publisher.o \
  agnocast_kunit_remove_subscriber.o \
  agnocast_kunit_remove_publisher.o \
  agnocast_kunit_release_sub_ref.o \
  agnocast_kunit_publish_msg.o \
  agnocast_kunit_receive_msg.o \
  agnocast_kunit_take_msg.o \
  agnocast_kunit_get_publisher_num.o \
  agnocast_kunit_get_publisher_qos.o \
  agnocast_kunit_get_subscriber_num.o \
  agnocast_kunit_get_subscriber_qos.o \
  agnocast_kunit_add_bridge.o \
  agnocast_kunit_remove_bridge.o \
  agnocast_kunit_set_ros2_subscriber_num.o \
  agnocast_kunit_set_ros2_publisher_num.o \
  agnocast_kunit_do_exit.o \
  agnocast_kunit_get_node_subscriber_topics.o \
  agnocast_kunit_get_node_publisher_topics.o \
  agnocast_kunit_get_version.o \
  agnocast_kunit_bridge_shutdown.o \
  agnocast_kunit_check_and_request_bridge_shutdown.o \
  agnocast_kunit_get_topic_subscriber_info.o \
  agnocast_kunit_get_topic_publisher_info.o \
  agnocast_kunit_init_memory_allocator.o

agnocast_kunit-objs += $(addprefix agnocast_kunit/, $(KUNIT_TEST_OBJS))
