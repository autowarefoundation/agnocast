^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agnocast_e2e_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.3 (2026-04-02)
------------------

2.3.2 (2026-03-24)
------------------

2.3.1 (2026-03-17)
------------------

2.3.0 (2026-03-09)
------------------
* fix(bridge): create bridge with checking ros2 pub/sub (`#1164 <https://github.com/autowarefoundation/agnocast/issues/1164>`_)
* fix(components): move tests for component container in `agnocast_components` (`#1126 <https://github.com/autowarefoundation/agnocast/issues/1126>`_)
* fix(scripts): add shebang and file extension (`#1133 <https://github.com/autowarefoundation/agnocast/issues/1133>`_)
* test: parallel execution mode available in e2e_test 1to1 (`#1092 <https://github.com/autowarefoundation/agnocast/issues/1092>`_)

2.2.0 (2026-02-19)
------------------
* fix e2e test (`#1075 <https://github.com/tier4/agnocast/issues/1075>`_)
* fix: delete deprecated agnocast mempool size (`#1068 <https://github.com/tier4/agnocast/issues/1068>`_)
* feat(kmod)[need-minor-update]: increase MAX_PUBLISHER_NUM (`#1060 <https://github.com/tier4/agnocast/issues/1060>`_)
* feat(agnocast_e2e_test): add r2a and a2r test (`#983 <https://github.com/tier4/agnocast/issues/983>`_)
* fix(agnocast_e2e_test): 1to1 e2e test failure when Bridge Mode is off (`#1008 <https://github.com/tier4/agnocast/issues/1008>`_)
* feat(agnocast_e2e_test): add r2a bridge test (`#970 <https://github.com/tier4/agnocast/issues/970>`_)
* fix(agnocast_e2e_test): verification logic (`#957 <https://github.com/tier4/agnocast/issues/957>`_)
* feat(agnocastlib)[need-minor-update]: add get_intra_subscription_count api (`#934 <https://github.com/tier4/agnocast/issues/934>`_)
* feat(agnocastlib): enable bridge function (`#872 <https://github.com/tier4/agnocast/issues/872>`_)

2.1.2 (2025-08-18)
------------------

2.1.1 (2025-05-13)
------------------
* fix(e2e_test): extend ready_delay waiting time (`#630 <https://github.com/tier4/agnocast/issues/630>`_)
* fix(e2e test): extend waiting time for receiving transient local messages (`#628 <https://github.com/tier4/agnocast/issues/628>`_)

2.1.0 (2025-04-15)
------------------

2.0.1 (2025-04-03)
------------------

2.0.0 (2025-04-02)
------------------
* fix: use AGNOCAST prefix for MEMPOOL_SIZE (`#580 <https://github.com/tier4/agnocast/issues/580>`_)
* fix: e2e test in restricted environments in terms of the number of cores (`#539 <https://github.com/tier4/agnocast/issues/539>`_)

1.0.2 (2025-03-14)
------------------
* feat: add validation if agnocast cb and ros2 cb belong to same MutuallyExclusive cbg in MultiThreadedAgnocastExecutor (`#515 <https://github.com/tier4/agnocast/issues/515>`_)
* fix: stress test (`#526 <https://github.com/tier4/agnocast/issues/526>`_)
* improve: stress test (`#522 <https://github.com/tier4/agnocast/issues/522>`_)
* fix: remove launch_testing_ament_cmake (`#513 <https://github.com/tier4/agnocast/issues/513>`_)

1.0.1 (2025-03-10)
------------------
* fix: not to check display order in e2e tests (`#475 <https://github.com/tier4/agnocast/issues/475>`_)

1.0.0 (2024-03-03)
------------------
* First release.
