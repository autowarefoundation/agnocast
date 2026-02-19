^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package agnocast_sample_application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.0 (2026-02-19)
------------------
* fix: delete deprecated agnocast mempool size (`#1068 <https://github.com/tier4/agnocast/issues/1068>`_)
* fix(sample_app): add missing agnocast_components dependency to package.xml (`#1029 <https://github.com/tier4/agnocast/issues/1029>`_)
* feat(agnocast_components): add agnocast_components_register_node macro (`#1025 <https://github.com/tier4/agnocast/issues/1025>`_)
* feat(sample_app): implement service/client for agnocast::Node (`#1012 <https://github.com/tier4/agnocast/issues/1012>`_)
* feat(agnocastlib): add generic_timer (`#978 <https://github.com/tier4/agnocast/issues/978>`_)
* feat(agnocastlib): add agnocast::Node support for PollingSubscriber   (`#926 <https://github.com/tier4/agnocast/issues/926>`_)
* fix(agnocastlib): fix create_wall_timer api to align with rclcpp (`#950 <https://github.com/tier4/agnocast/issues/950>`_)
* feat(agnocastlib): add agnocast timer implementation (`#942 <https://github.com/tier4/agnocast/issues/942>`_)
* feat(cie_thread_configurator): support multiple ROS domain (`#951 <https://github.com/tier4/agnocast/issues/951>`_)
* feat(agnocastlib): simple NodeTimeSource implementation (`#924 <https://github.com/tier4/agnocast/issues/924>`_)
* feat(agnocastlib): add NodeClock (`#912 <https://github.com/tier4/agnocast/issues/912>`_)
* feat(agnocastlib): use qos parameter overrides (`#905 <https://github.com/tier4/agnocast/issues/905>`_)
* feat(agnocastlib): add parameter recursive mutation guard (`#894 <https://github.com/tier4/agnocast/issues/894>`_)
* feat(agnocastlib): add get_parameters overload (`#896 <https://github.com/tier4/agnocast/issues/896>`_)
* feat(agnocastlib): add get_parameters_by_prefix implementation (`#895 <https://github.com/tier4/agnocast/issues/895>`_)
* feat(agnocastlib): run parameter callback when declare (`#892 <https://github.com/tier4/agnocast/issues/892>`_)
* feat(agnocastlib): add set_parameters_callback impl (`#890 <https://github.com/tier4/agnocast/issues/890>`_)
* feat(agnocastlib):add set_parameter implementation and sample app (`#889 <https://github.com/tier4/agnocast/issues/889>`_)
* feat(agnocastlib): support agnocast::Node publisher (`#873 <https://github.com/tier4/agnocast/issues/873>`_)
* fix(agnocastlib): delete resolve_topic_name from agnocast::Node (`#875 <https://github.com/tier4/agnocast/issues/875>`_)
* fix(agnocastlib): apply resolve_topic_name (`#871 <https://github.com/tier4/agnocast/issues/871>`_)
* refactor(agnocastlib): restore resolve_topic_name impl (`#868 <https://github.com/tier4/agnocast/issues/868>`_)
* feat(agnocastlib): declare/has/get_parameter functions support for agnocast::Node (`#853 <https://github.com/tier4/agnocast/issues/853>`_)
* fix(sample_application): remove unnecessary test (`#852 <https://github.com/tier4/agnocast/issues/852>`_)
* fix(sample_application): fix use_remap condition of no_rclcpp_listener launch (`#817 <https://github.com/tier4/agnocast/issues/817>`_)
* feat(agnocastlib): add node_name/namespace remapping (`#777 <https://github.com/tier4/agnocast/issues/777>`_)
* feat(agnocastlib): add CallbackGroup related NodeBaseInterface function implementations (`#779 <https://github.com/tier4/agnocast/issues/779>`_)
* feat(agnocastlib): add node base interface (`#773 <https://github.com/tier4/agnocast/issues/773>`_)
* feat(agnocastlib): add agnocast-only executor (`#760 <https://github.com/tier4/agnocast/issues/760>`_)
* feat(agnocastlib): agnocast::Node create_subscription (`#758 <https://github.com/tier4/agnocast/issues/758>`_)
* feat(agnocastlib): add agnocast::Node (`#756 <https://github.com/tier4/agnocast/issues/756>`_)
* feat(agnocastlib): add agnocast::init as public API (`#745 <https://github.com/tier4/agnocast/issues/745>`_)
* feat(sample_application): add no rclcpp application skelton (`#739 <https://github.com/tier4/agnocast/issues/739>`_)
* fix(kmod) [needs minor version update]: Remove AGNOCAST_MEMPOOL_SIZE (`#719 <https://github.com/tier4/agnocast/issues/719>`_)
* feat:: port component_container for callback_isolated_executor (`#721 <https://github.com/tier4/agnocast/issues/721>`_)
* chore: add sample service/client application (`#709 <https://github.com/tier4/agnocast/issues/709>`_)
* fix(agnocast_sample_application): refine cie sample application (`#681 <https://github.com/tier4/agnocast/issues/681>`_)

2.1.2 (2025-08-18)
------------------
* feat(agnocastlib): cie spin (`#669 <https://github.com/tier4/agnocast/issues/669>`_)
* fix(agnocastlib): stop possessing shared pointer to nodes in agnocast executor (`#664 <https://github.com/tier4/agnocast/issues/664>`_)

2.1.1 (2025-05-13)
------------------

2.1.0 (2025-04-15)
------------------

2.0.1 (2025-04-03)
------------------

2.0.0 (2025-04-02)
------------------
* fix: use AGNOCAST prefix for MEMPOOL_SIZE (`#580 <https://github.com/tier4/agnocast/issues/580>`_)

1.0.2 (2025-03-14)
------------------

1.0.1 (2025-03-10)
------------------
* chore: remove callback_group_test in sample (`#461 <https://github.com/tier4/agnocast/issues/461>`_)

1.0.0 (2024-03-03)
------------------
* First release.
