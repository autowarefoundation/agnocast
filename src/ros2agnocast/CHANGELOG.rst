^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2agnocast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.5 (2026-06-09)
------------------
* Feat/ros2 bag record agnocast/topic filter (`#1377 <https://github.com/autowarefoundation/agnocast/issues/1377>`_)
* refactor(ros2agnocast): reorganize verb directory and extract common logic (`#1374 <https://github.com/autowarefoundation/agnocast/issues/1374>`_)
* feat(ros2agnocast): add `ros2 bag record_agnocast` CLI verb (`#1373 <https://github.com/autowarefoundation/agnocast/issues/1373>`_)
* feat(ros2agnocast): add ros2agnocast verb `ros2 agnocast bridge-daemo… (`#1369 <https://github.com/autowarefoundation/agnocast/issues/1369>`_)
* perf(ros2agnocast): return from gossip collection once all publishers report (`#1370 <https://github.com/autowarefoundation/agnocast/issues/1370>`_)
* feat(ros2agnocast): add discovery_daemon_status verb for daemon liveness (`#1363 <https://github.com/autowarefoundation/agnocast/issues/1363>`_)
* feat(ros2agnocast): support ros2 topic hz/echo/delay for Agnocast (`#1357 <https://github.com/autowarefoundation/agnocast/issues/1357>`_)
* feat: ros2agnocast CLI verbs see Agnocast endpoints across IPC namespaces and ECUs (`#1351 <https://github.com/autowarefoundation/agnocast/issues/1351>`_)

2.3.4 (2026-05-21)
------------------
* fix(bridge): suppress bridge plugin compilation warning (`#1347 <https://github.com/autowarefoundation/agnocast/issues/1347>`_)
* fix: ubuntu24.04/jazzy compilation warnings (`#1339 <https://github.com/autowarefoundation/agnocast/issues/1339>`_)
* perf(bridge-plugins): make bridge nodes use generic publisher and subscriber (`#1337 <https://github.com/autowarefoundation/agnocast/issues/1337>`_)
* refactor(bridge-plugins): consolidate .so and enable unity/lld build (`#1336 <https://github.com/autowarefoundation/agnocast/issues/1336>`_)
* feat(bridge): introduce shadow node to Agnocast bridge (`#1300 <https://github.com/autowarefoundation/agnocast/issues/1300>`_)
* enhance(ros2agnocas): use `get_agnocast_node_topics` in `node info_agnocast` (`#1237 <https://github.com/autowarefoundation/agnocast/issues/1237>`_)
* feat(bridge): add standard-mode R2A service bridge support (`#1289 <https://github.com/autowarefoundation/agnocast/issues/1289>`_)
* enhance(ros2agnocast): avoid unnecessary `get_topic_names_and_types` in `topic info_agnocast` (`#1236 <https://github.com/autowarefoundation/agnocast/issues/1236>`_)
* refactor(ros2agnocast): typo, memory handle (`#1305 <https://github.com/autowarefoundation/agnocast/issues/1305>`_)
* refactor(ros2agnocast): use context manager for ctypes topic arrays (`#1293 <https://github.com/autowarefoundation/agnocast/issues/1293>`_)
* feat(bridge): add e2e support for R2A service bridges in performance mode (`#1274 <https://github.com/autowarefoundation/agnocast/issues/1274>`_)
* enhance(ros2agnocast): avoid unnecessary `get_publishers/subscriptions_info_by_topic` calls in `topic list_agnocast` (`#1235 <https://github.com/autowarefoundation/agnocast/issues/1235>`_)
* fix(bridge): inherit version of ros2agnocast for agnocast bridge plugins (`#1219 <https://github.com/autowarefoundation/agnocast/issues/1219>`_)
* feat:add version check command (`#1224 <https://github.com/autowarefoundation/agnocast/issues/1224>`_)
* feat(ros2agnocast): add R2A service plugin template (`#1144 <https://github.com/autowarefoundation/agnocast/issues/1144>`_)

2.3.3 (2026-04-02)
------------------
* fix: topic list_agnocast warning (`#1208 <https://github.com/autowarefoundation/agnocast/issues/1208>`_)

2.3.2 (2026-03-24)
------------------
* refactor(bridge): use CallbackIsolatedAgnocastExecutor for bridge callback scheduling (`#1170 <https://github.com/autowarefoundation/agnocast/issues/1170>`_)
* fix(ros2agnocast): show Agnocast label for topics that exist in both ROS2 and Agnocast (`#1180 <https://github.com/autowarefoundation/agnocast/issues/1180>`_)

2.3.1 (2026-03-17)
------------------

2.3.0 (2026-03-09)
------------------

2.2.0 (2026-02-19)
------------------
* fix(bridge): unify r2a and a2r plugin into one to improve build time (`#1064 <https://github.com/tier4/agnocast/issues/1064>`_)
* fix(bridge): improve bridge plugins build time (`#1063 <https://github.com/tier4/agnocast/issues/1063>`_)
* feat(ros2agnocast): change bridge display meaning (`#1005 <https://github.com/tier4/agnocast/issues/1005>`_)
* fix email address of package.xml. template (`#1024 <https://github.com/tier4/agnocast/issues/1024>`_)
* feat(ros2agnocast): enhance node info agnocast command (`#1004 <https://github.com/tier4/agnocast/issues/1004>`_)
* feat: redesign plugin generation process of performance bridge (`#1006 <https://github.com/tier4/agnocast/issues/1006>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast): add node list agnocast command (`#1003 <https://github.com/tier4/agnocast/issues/1003>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast): enhance topic list agnocast command (`#1002 <https://github.com/tier4/agnocast/issues/1002>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast): mode print feature topic list (`#992 <https://github.com/tier4/agnocast/issues/992>`_)
* refactor(agnocastlib)[needs minor version update]: add debug mode for bridge (`#963 <https://github.com/tier4/agnocast/issues/963>`_)
* fix(ros2agnocast): account for topic names used by Agnocast services (`#712 <https://github.com/tier4/agnocast/issues/712>`_)

2.1.2 (2025-08-18)
------------------

2.1.1 (2025-05-13)
------------------

2.1.0 (2025-04-15)
------------------

2.0.1 (2025-04-03)
------------------

2.0.0 (2025-04-02)
------------------

1.0.2 (2025-03-14)
------------------

1.0.1 (2025-03-10)
------------------
* feat(ros2agnocast): add topic info cmd base (`#442 <https://github.com/tier4/agnocast/issues/442>`_)

1.0.0 (2024-03-03)
------------------
* First release.
