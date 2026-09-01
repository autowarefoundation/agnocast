^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2agnocast_discovery_agent
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix(ros2agnocast_discovery_agent): resolve domain bridge config topic names to absolute names (`#1578 <https://github.com/tier4/agnocast/issues/1578>`_)
* fix(agnocastlib): break the cross-IPC-namespace service bridge deadlock (`#1552 <https://github.com/tier4/agnocast/issues/1552>`_)
* Fix/daemon force cross domain bridge (`#1574 <https://github.com/tier4/agnocast/issues/1574>`_)
* perf(agnocastlib)[need-minor-update]: exec the discovery agent directly instead of via ros2 run (`#1473 <https://github.com/tier4/agnocast/issues/1473>`_)
* refactor(agnocast)[need-minor-update]: remove the subscriber MQ machinery eventfd made dead (`#1503 <https://github.com/tier4/agnocast/issues/1503>`_)
* fix(ros2agnocast_discovery_agent): log the idle exit at debug level (`#1512 <https://github.com/tier4/agnocast/issues/1512>`_)
* refactor(ros2agnocast_discovery_agent): rename the executable to agnocast_discovery_agent (`#1496 <https://github.com/tier4/agnocast/issues/1496>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): register renamed domain bridge rules from the YAML remap field (`#1437 <https://github.com/tier4/agnocast/issues/1437>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): claim the discovery-agent singleton via the kmod (`#1427 <https://github.com/tier4/agnocast/issues/1427>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): register domain bridge rules from YAML (`#1418 <https://github.com/tier4/agnocast/issues/1418>`_)
* refactor(agnocastlib): replace bridge MQ with UDS (`#1433 <https://github.com/tier4/agnocast/issues/1433>`_)
* refactor(agnocastlib): merge two MQs of the bridge manager into one (`#1431 <https://github.com/tier4/agnocast/issues/1431>`_)
* fix(ros2agnocast_discovery_agent): scope each agent's discovery to its own domain (`#1415 <https://github.com/tier4/agnocast/issues/1415>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): make cross-NS discovery and bridge decisions domain-aware (`#1410 <https://github.com/tier4/agnocast/issues/1410>`_)
* fix(ros2agnocast_discovery_agent): dedup the discovery agent per (IPC namespace, ROS_DOMAIN_ID) (`#1411 <https://github.com/tier4/agnocast/issues/1411>`_)
* feat(agnocastlib, ros2agnocast_discovery_agent): add cross-NS daemon bridge MQ and let the discovery agent dispatch bridge requests (`#1389 <https://github.com/tier4/agnocast/issues/1389>`_)

2.3.5 (2026-06-09)
------------------
* fix(ros2agnocast_discovery_agent): exit duplicate agent cleanly instead of signal.pause() (`#1380 <https://github.com/autowarefoundation/agnocast/issues/1380>`_)
* feat(ros2agnocast_discovery_agent,agnocastlib): publish Agnocast endpoint types in gossip via tmpfs registry (`#1356 <https://github.com/autowarefoundation/agnocast/issues/1356>`_)
* fix(ros2agnocast_discovery_agent): add pytest for test_agent (`#1360 <https://github.com/autowarefoundation/agnocast/issues/1360>`_)
* feat: per-IPC-namespace discovery daemon + AgnocastDaemonState msgs (shared base) (`#1350 <https://github.com/autowarefoundation/agnocast/issues/1350>`_)
