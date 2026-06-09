^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agnocast_components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.5 (2026-06-09)
------------------

2.3.4 (2026-05-21)
------------------
* fix(cie): fix cancel_executor race condition (`#1278 <https://github.com/autowarefoundation/agnocast/issues/1278>`_)
* refactor(agnocast_components): use rclcpp_components_register_nodes for resource index registration (`#1238 <https://github.com/autowarefoundation/agnocast/issues/1238>`_)
* fix(agnocast_components): allow agnocast_components_register_node to coexist with rclcpp_components_register_node (`#1230 <https://github.com/autowarefoundation/agnocast/issues/1230>`_)
* refactor(e2e_test): move CIE component container test to agnocast_e2e_test (`#1228 <https://github.com/autowarefoundation/agnocast/issues/1228>`_)
* fix(test): filter out bridge node from callback info count in CIE test (`#1226 <https://github.com/autowarefoundation/agnocast/issues/1226>`_)
* refactor(test): replace assertSequentialStdout with direct proc_output access (`#1225 <https://github.com/autowarefoundation/agnocast/issues/1225>`_)

2.3.3 (2026-04-02)
------------------
* feat(agnocastlib): enable file logging for `agnocast::Node` (`#1204 <https://github.com/autowarefoundation/agnocast/issues/1204>`_)

2.3.2 (2026-03-24)
------------------

2.3.1 (2026-03-17)
------------------

2.3.0 (2026-03-09)
------------------
* fix(components): move tests for component container in `agnocast_components` (`#1126 <https://github.com/autowarefoundation/agnocast/issues/1126>`_)
* fix: prevent signed integer overflow in ms-to-ns conversion (`#1136 <https://github.com/autowarefoundation/agnocast/issues/1136>`_)
* feat(agnocast_component_container_cie): support callback group created after spin() (`#1095 <https://github.com/autowarefoundation/agnocast/issues/1095>`_)

2.2.0 (2026-02-19)
------------------
* feat(components): move component container to agnocast_components package (`#1074 <https://github.com/tier4/agnocast/issues/1074>`_)
* fix(components):  fix to use wrapper node base (`#1057 <https://github.com/tier4/agnocast/issues/1057>`_)
* feat(agnocastlib): add AgnocastOnlyCallbackIsolatedExecutor (`#1031 <https://github.com/tier4/agnocast/issues/1031>`_)
* feat(agnocastlib, agnocast_components): add glog initialization to component containers and node templates (`#1026 <https://github.com/tier4/agnocast/issues/1026>`_)
* feat(agnocast_components): add agnocast_components_register_node macro (`#1025 <https://github.com/tier4/agnocast/issues/1025>`_)

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
