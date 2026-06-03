from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension

from ros2agnocast.discovery import (
    add_gossip_timeout_arg,
    all_topic_names,
    bridge_label_from_roles,
    BRIDGE_LABEL_TEXT,
    collect_announcements_with_fallback,
    collect_bridge_roles,
    warn_if_gossip_timeout_overridden,
    warn_if_using_fallback,
)

class ListAgnocastVerb(VerbExtension):
    "Output a list of available topics including Agnocast"

    def add_arguments(self, parser, cli_name):
        add_gossip_timeout_arg(parser)

    def main(self, *, args):
        warn_if_gossip_timeout_overridden(args)
        with NodeStrategy(None) as node:
            snapshots, used_fallback = collect_announcements_with_fallback(
                node, timeout_sec=args.gossip_timeout)
            warn_if_using_fallback(node, used_fallback, args.gossip_timeout)
            bridge_roles = collect_bridge_roles(snapshots)

            def divide_ros2_topic_into_pubsub(topic_names):
                pub_topics = []
                sub_topics = []
                for name in topic_names:
                    pubs_info = node.get_publishers_info_by_topic(name)
                    subs_info = node.get_subscriptions_info_by_topic(name)

                    # Remove Agnocast bridge nodes from the list
                    pubs_info = [info for info in pubs_info if not info.node_name.startswith("agnocast_bridge_node_")]
                    subs_info = [info for info in subs_info if not info.node_name.startswith("agnocast_bridge_node_")]

                    if pubs_info:
                        pub_topics.append(name)
                    if subs_info:
                        sub_topics.append(name)
                return pub_topics, sub_topics

            def remove_service_topic(topic_names):
                return [name for name in topic_names if not name.startswith('/AGNOCAST_SRV_')]

            # Get Agnocast topics from gossip (every NS / ECU on the domain).
            agnocast_topics = remove_service_topic(list(all_topic_names(snapshots)))

            # Get ros2 topics
            ros2_topics_data = get_topic_names_and_types(node=node)
            ros2_all_topics = set(name for name, _ in ros2_topics_data)

            ########################################################################
            # Print topic list
            ########################################################################
            agnocast_topics_set = set(agnocast_topics)

            # Non-agnocast ROS2 topics cannot have bridge nodes, so no filtering needed.
            ros2_only_topics = ros2_all_topics - agnocast_topics_set
            # Only query pub/sub breakdown for topics in both sets (expensive ROS2 API calls).
            overlapping_candidates = list(agnocast_topics_set & ros2_all_topics)
            ros2_pub_topics, ros2_sub_topics = divide_ros2_topic_into_pubsub(overlapping_candidates)
            ros2_pub_set = set(ros2_pub_topics)
            ros2_sub_set = set(ros2_sub_topics)
            ros2_topics_set = ros2_only_topics | ros2_pub_set | ros2_sub_set

            for topic in sorted(agnocast_topics_set | ros2_topics_set):
                if topic in agnocast_topics_set:
                    status = bridge_label_from_roles(
                        bridge_roles.get(topic, []), topic in ros2_pub_set, topic in ros2_sub_set)
                    suffix = f" ({BRIDGE_LABEL_TEXT[status]})"
                else:
                    suffix = ""
                print(f"{topic}{suffix}")
