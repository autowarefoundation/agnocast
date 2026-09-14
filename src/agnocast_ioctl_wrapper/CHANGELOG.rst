^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agnocast_ioctl_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2026-09-01)
------------------
* perf(agnocastlib)[need-minor-update]: exec the discovery agent directly instead of via ros2 run (`#1473 <https://github.com/tier4/agnocast/issues/1473>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): register renamed domain bridge rules from the YAML remap field (`#1437 <https://github.com/tier4/agnocast/issues/1437>`_)
* feat(agnocast_kmod)[needs minor version update]: support cross-domain topic rename in the domain bridge (`#1436 <https://github.com/tier4/agnocast/issues/1436>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): claim the discovery-agent singleton via the kmod (`#1427 <https://github.com/tier4/agnocast/issues/1427>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): register domain bridge rules from YAML (`#1418 <https://github.com/tier4/agnocast/issues/1418>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast_discovery_agent): make cross-NS discovery and bridge decisions domain-aware (`#1410 <https://github.com/tier4/agnocast/issues/1410>`_)
* feat(agnocast_kmod, agnocast_ioctl_wrapper)[needs minor version update]: make the kmod domain-aware (topic ioctls + bridge-manager spawn) (`#1409 <https://github.com/tier4/agnocast/issues/1409>`_)

2.3.5 (2026-06-09)
------------------

2.3.4 (2026-05-21)
------------------
* refactor(ros2agnocast): typo, memory handle (`#1305 <https://github.com/autowarefoundation/agnocast/issues/1305>`_)
* refactor(ros2agnocast): use context manager for ctypes topic arrays (`#1293 <https://github.com/autowarefoundation/agnocast/issues/1293>`_)
* chore: remove @veqcc from maintainers (`#1296 <https://github.com/autowarefoundation/agnocast/issues/1296>`_)
* feat:add version check command (`#1224 <https://github.com/autowarefoundation/agnocast/issues/1224>`_)

2.3.3 (2026-04-02)
------------------
* fix: topic list_agnocast warning (`#1208 <https://github.com/autowarefoundation/agnocast/issues/1208>`_)

2.3.2 (2026-03-24)
------------------

2.3.1 (2026-03-17)
------------------

2.3.0 (2026-03-09)
------------------
* feat(kmod): bump capacity constants and default mempool size (`#1151 <https://github.com/autowarefoundation/agnocast/issues/1151>`_)
* fix(kmod)[need-minor-update]: add `topic_name_buffer_size` field for `get_agnocast_pub/sub_topics` (`#1113 <https://github.com/autowarefoundation/agnocast/issues/1113>`_)

2.2.0 (2026-02-19)
------------------
* feat(kmod)[need-minor-update]: increase MAX_PUBLISHER_NUM (`#1060 <https://github.com/tier4/agnocast/issues/1060>`_)
* feat(kmod): Increase MAX_SUBSCRIBER_NUM by using bitmap (`#1015 <https://github.com/tier4/agnocast/issues/1015>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast): enhance topic list agnocast command (`#1002 <https://github.com/tier4/agnocast/issues/1002>`_)
* feat(agnocast_ioctl_wrapper, ros2agnocast): mode print feature topic list (`#992 <https://github.com/tier4/agnocast/issues/992>`_)
* refactor(agnocastlib)[needs minor version update]: add debug mode for bridge (`#963 <https://github.com/tier4/agnocast/issues/963>`_)
* fix(agnocastlib): error output when opening agnocast driver failed (`#732 <https://github.com/tier4/agnocast/issues/732>`_)
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
* refactor(kmod): unified magic number for ioctl (`#569 <https://github.com/tier4/agnocast/issues/569>`_)
* fix: pass string length along with its pointer (`#534 <https://github.com/tier4/agnocast/issues/534>`_)

1.0.2 (2025-03-14)
------------------

1.0.1 (2025-03-10)
------------------
* fix(ioctl_wrapper): remove NOLINT (`#477 <https://github.com/tier4/agnocast/issues/477>`_)
* feat(ros2agnocast): add topic info cmd base (`#442 <https://github.com/tier4/agnocast/issues/442>`_)

1.0.0 (2024-03-03)
------------------
* First release.
