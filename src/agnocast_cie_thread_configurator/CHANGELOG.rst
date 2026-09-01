^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agnocast_cie_thread_configurator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* refactor(agnocast_cie_thread_configurator): share the manageable CPU bound between parser and scanner (`#1589 <https://github.com/tier4/agnocast/issues/1589>`_)
* refactor(agnocast_cie_thread_configurator): move is_kworker_comm to system_scan (`#1588 <https://github.com/tier4/agnocast/issues/1588>`_)
* refactor(agnocast_cie_thread_configurator): replace policy strings with a SchedPolicy enum (`#1587 <https://github.com/tier4/agnocast/issues/1587>`_)
* refactor(agnocast_cie_thread_configurator): split daemon-internal logic into a static core library (`#1586 <https://github.com/tier4/agnocast/issues/1586>`_)
* refactor(agnocast_cie_thread_configurator): own the listener's file descriptors with UniqueFd (`#1585 <https://github.com/tier4/agnocast/issues/1585>`_)
* refactor(agnocast_cie_thread_configurator): format int64_t thread ids with PRId64 (`#1584 <https://github.com/tier4/agnocast/issues/1584>`_)
* chore(agnocast_cie_thread_configurator): name the IPC gtest target after its contents (`#1583 <https://github.com/tier4/agnocast/issues/1583>`_)
* feat(agnocast_cie_thread_configurator): apply kernel thread and IRQ configs with reapply support (`#1546 <https://github.com/tier4/agnocast/issues/1546>`_)
* feat(agnocast_cie_thread_configurator): emit kernel_threads and irqs sections in prerun template (`#1545 <https://github.com/tier4/agnocast/issues/1545>`_)
* feat(agnocast_cie_thread_configurator): parse kernel_threads and irqs config sections (`#1544 <https://github.com/tier4/agnocast/issues/1544>`_)
* feat(agnocast_cie_thread_configurator): add system scan utilities for kernel threads and IRQs (`#1543 <https://github.com/tier4/agnocast/issues/1543>`_)
* fix(agnocast_cie_thread_configurator): validate and normalize affinity CPU lists (`#1538 <https://github.com/tier4/agnocast/issues/1538>`_)
* fix(agnocast_cie_thread_configurator): take 'nice' instead of 'priority' for CFS policies (`#1529 <https://github.com/tier4/agnocast/issues/1529>`_)
* feat(agnocast_cie_thread_configurator): support wildcard callback group ids (<node name>/*) in the config YAML (`#1454 <https://github.com/tier4/agnocast/issues/1454>`_)
* refactor(cie_thread_configurator): remove remove_trailing_waitable (`#1472 <https://github.com/tier4/agnocast/issues/1472>`_)

2.3.5 (2026-06-09)
------------------
* feat(cie_thread_configurator): add pure-Rust non-ROS thread notification client (`#1371 <https://github.com/autowarefoundation/agnocast/issues/1371>`_)

2.3.4 (2026-05-21)
------------------
* fix: ubuntu24.04/jazzy compilation warnings (`#1339 <https://github.com/autowarefoundation/agnocast/issues/1339>`_)
* fix(sample_application): add agnocast\_ prefix to sample app and thread_configurator (`#1319 <https://github.com/autowarefoundation/agnocast/issues/1319>`_)
* feat(cie_thread_configurator): add ~/reapply_config service for runtime YAML reload (`#1303 <https://github.com/autowarefoundation/agnocast/issues/1303>`_)
* refactor(cie_thread_configurator): extract ThreadConfig and YAML parser into thread_config.hpp/cpp (`#1304 <https://github.com/autowarefoundation/agnocast/issues/1304>`_)
* refactor(cie_thread_configurator): migrate non-ROS thread reporting from rclcpp pub/sub to abstract Unix domain socket (`#1295 <https://github.com/autowarefoundation/agnocast/issues/1295>`_)
* refactor(cie_thread_configurator): split callback groups and atomicize shared state (`#1288 <https://github.com/autowarefoundation/agnocast/issues/1288>`_)
* fix(cie_thread_configurator): address misc code quality issues (`#1277 <https://github.com/autowarefoundation/agnocast/issues/1277>`_)
* fix(cie_thread_configurator): always re-apply configuration regardless of thread_id (`#1262 <https://github.com/autowarefoundation/agnocast/issues/1262>`_)
* refactor(cie_thread_configurator): apply SCHED_DEADLINE immediately with RESET_ON_FORK (`#1248 <https://github.com/autowarefoundation/agnocast/issues/1248>`_)
* fix: add missing export dependencies (`#1254 <https://github.com/autowarefoundation/agnocast/issues/1254>`_)
* refactor(thread_configurator): split into separate executables and use ROS parameters (`#1234 <https://github.com/autowarefoundation/agnocast/issues/1234>`_)
* fix(cie_thread_configurator): make spawn_non_ros2_thread node name unique using thread_name (`#1243 <https://github.com/autowarefoundation/agnocast/issues/1243>`_)
* fix(cie_thread_configurator): align QoS settings between ThreadConfiguratorNode and PrerunNode (`#1218 <https://github.com/autowarefoundation/agnocast/issues/1218>`_)

2.3.3 (2026-04-02)
------------------

2.3.2 (2026-03-24)
------------------
* dont define sched_attr for glibc 2.42 (`#1190 <https://github.com/autowarefoundation/agnocast/issues/1190>`_)

2.3.1 (2026-03-17)
------------------

2.3.0 (2026-03-09)
------------------

2.2.0 (2026-02-19)
------------------
* fix: change package names from cie\_* to agnocast_cie* (`#1071 <https://github.com/tier4/agnocast/issues/1071>`_)

2.1.2 (2025-08-19)
------------------

2.1.1 (2025-05-13)
------------------

2.1.0 (2025-04-16)
------------------

2.0.1 (2025-04-03)
------------------

2.0.0 (2025-04-02)
------------------

1.0.2 (2025-03-14)
------------------

1.0.1 (2025-03-10)
------------------

1.0.0 (2025-03-03)
------------------
