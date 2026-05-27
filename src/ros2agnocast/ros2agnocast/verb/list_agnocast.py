from enum import Enum
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension

from ros2agnocast.discovery import (
    add_gossip_timeout_arg,
    all_topic_names,
    collect_announcements_with_fallback,
    warn_if_gossip_timeout_overridden,
    warn_if_using_fallback,
)

class BridgeStatus(Enum):
    NONE = 0
    ROS2_TO_AGNOCAST = 1
    AGNOCAST_TO_ROS2= 2
    BIDIRECTION = 3

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

            def get_bridge_status(topic_name):
                has_sub_bridge = False
                has_pub_bridge = False
                has_agnocast_sub = False
                has_agnocast_pub = False

                for snap in snapshots:
                    for topic in snap.topics:
                        if topic.topic_name != topic_name:
                            continue
                        for n in topic.subscribers:
                            if n.is_bridge:
                                has_sub_bridge = True
                            else:
                                has_agnocast_sub = True
                        for n in topic.publishers:
                            if n.is_bridge:
                                has_pub_bridge = True
                            else:
                                has_agnocast_pub = True

                mapping = {
                    (True, True):   BridgeStatus.BIDIRECTION,
                    (True, False):  BridgeStatus.AGNOCAST_TO_ROS2,
                    (False, True):  BridgeStatus.ROS2_TO_AGNOCAST,
                    (False, False): BridgeStatus.NONE,
                }

                return mapping[(has_sub_bridge, has_pub_bridge)], has_agnocast_pub, has_agnocast_sub

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
            ros2_pub_topics_set = set(ros2_pub_topics)
            ros2_sub_topics_set = set(ros2_sub_topics)
            ros2_topics_set = ros2_only_topics | ros2_pub_topics_set | ros2_sub_topics_set

            for topic in sorted(agnocast_topics_set | ros2_topics_set):
                if topic in agnocast_topics_set and topic not in ros2_topics_set:
                    suffix = " (Agnocast enabled)"
                elif topic in ros2_topics_set and topic not in agnocast_topics_set:
                    suffix = ""
                else:
                    bridge_status, has_agnocast_pub, has_agnocast_sub = get_bridge_status(topic)
                    needs_r2a = has_agnocast_sub and topic in ros2_pub_topics_set
                    needs_a2r = has_agnocast_pub and topic in ros2_sub_topics_set
                    match bridge_status:
                        case BridgeStatus.BIDIRECTION:
                            suffix = " (Agnocast enabled, bridged)"
                        case BridgeStatus.ROS2_TO_AGNOCAST:
                            if needs_a2r:
                                suffix = " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"
                            else:
                                suffix = " (Agnocast enabled, bridged)"
                        case BridgeStatus.AGNOCAST_TO_ROS2:
                            if needs_r2a:
                                suffix = " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"
                            else:
                                suffix = " (Agnocast enabled, bridged)"
                        case BridgeStatus.NONE:
                            if needs_r2a or needs_a2r:
                                suffix = " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"
                            else:
                                suffix = " (Agnocast enabled)"
                print(f"{topic}{suffix}")
