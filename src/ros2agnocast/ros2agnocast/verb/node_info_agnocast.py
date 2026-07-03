from ros2cli.node.strategy import NodeStrategy
from ros2node.api import (
    get_action_client_info, get_action_server_info, get_node_names,
    get_publisher_info, get_service_client_info, get_service_server_info, get_subscriber_info
)
from ros2topic.api import get_topic_names_and_types
from ros2node.verb import VerbExtension

from ros2agnocast.discovery import (
    add_gossip_timeout_arg,
    bridge_label_from_roles,
    BRIDGE_LABEL_TEXT,
    collect_announcements_with_fallback,
    collect_bridge_roles,
    warn_if_gossip_timeout_overridden,
    warn_if_using_fallback,
)

def service_name_from_request_topic(topic_name):
    prefix = '/AGNOCAST_SRV_REQUEST'
    if not topic_name.startswith(prefix):
        return None
    return topic_name[len(prefix):]

def service_name_from_response_topic(topic_name):
    prefix = '/AGNOCAST_SRV_RESPONSE'
    if not topic_name.startswith(prefix):
        return None
    return topic_name[len(prefix):].split('_SEP_')[0]

class NodeInfoAgnocastVerb(VerbExtension):
    "Output information about a node including Agnocast"

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'node_name',
            help='Fully qualified node name to request information with Agnocast topics')
        add_gossip_timeout_arg(parser)

    def main(self, *, args):
        warn_if_gossip_timeout_overridden(args)
        node_name = args.node_name

        with NodeStrategy(None) as node:
            snapshots, used_fallback = collect_announcements_with_fallback(
                node, timeout_sec=args.gossip_timeout)
            warn_if_using_fallback(node, used_fallback, args.gossip_timeout, snapshots)
            bridge_roles = collect_bridge_roles(snapshots)

            def get_agnocast_label(topic_name, ros2_sub_topics, ros2_pub_topics):
                """Get the appropriate label for an Agnocast-enabled topic."""
                status = bridge_label_from_roles(
                    bridge_roles.get(topic_name, []),
                    topic_name in ros2_pub_topics, topic_name in ros2_sub_topics)
                return f"({BRIDGE_LABEL_TEXT[status]})"

            def get_agnocast_node_topics(target_node_name):
                sub_topic_list = []
                pub_topic_list = []
                # service_name_from_response_topic strips the _SEP_<id> suffix,
                # so multiple response topics can collapse to the same service name.
                server_set = set()
                client_set = set()
                # type_name resolved from gossip, keyed by topic name.
                topic_types = {}

                for snap in snapshots:
                    for topic in snap.topics:
                        is_sub = any(ep.node_name == target_node_name for ep in topic.subscribers)
                        is_pub = any(ep.node_name == target_node_name for ep in topic.publishers)
                        if not (is_sub or is_pub):
                            continue

                        service_name = service_name_from_request_topic(topic.topic_name)
                        if service_name is not None:
                            if is_sub:
                                server_set.add(service_name)
                            continue

                        service_name = service_name_from_response_topic(topic.topic_name)
                        if service_name is not None:
                            if is_sub:
                                client_set.add(service_name)
                            continue

                        if is_sub and topic.topic_name not in sub_topic_list:
                            sub_topic_list.append(topic.topic_name)
                        if is_pub and topic.topic_name not in pub_topic_list:
                            pub_topic_list.append(topic.topic_name)
                        if topic.type_name:
                            topic_types[topic.topic_name] = topic.type_name

                return sub_topic_list, pub_topic_list, server_set, client_set, topic_types

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

            # Get Agnocast node info from gossip (every NS / ECU on the domain).
            agnocast_subscribers, agnocast_publishers, agnocast_servers, agnocast_clients, gossip_topic_types = get_agnocast_node_topics(node_name)

            # Get ros2 all node names
            ros2_node_name_list = get_node_names(node=node, include_hidden_nodes=True)
            ros2_node_names = {n.full_name for n in ros2_node_name_list}

            ########################################################################
            # Print node info
            ########################################################################
            subscribers = []
            publishers = []
            service_servers = []
            service_clients = []
            action_servers = []
            action_clients = []

            # Determine node class
            # 1. ros2 node
            if node_name in ros2_node_names:
                subscribers = get_subscriber_info(node=node, remote_node_name=node_name)
                publishers = get_publisher_info(node=node, remote_node_name=node_name)
                service_servers = get_service_server_info(node=node, remote_node_name=node_name)
                service_clients = get_service_client_info(node=node, remote_node_name=node_name)
                action_servers = get_action_server_info(node=node, remote_node_name=node_name)
                action_clients = get_action_client_info(node=node, remote_node_name=node_name)
            # 2. agnocast node
            elif any([agnocast_subscribers, agnocast_publishers, agnocast_servers, agnocast_clients]):
                pass
            # 3. unknown node
            else:
                print(f"Error: The node '{node_name}' does not exist.")
                return

            ros2_topic_raw = get_topic_names_and_types(node=node)
            ros2_topic_dir = [{'name': topic_name, 'types': topic_types} for topic_name, topic_types in ros2_topic_raw]
            ros2_topic_name_set = set(topic_name for topic_name, _ in ros2_topic_raw)

            # Only query pub/sub breakdown for agnocast topics that also exist in ROS2
            agnocast_all_topics = set(agnocast_subscribers) | set(agnocast_publishers)
            overlapping_topics = list(agnocast_all_topics & ros2_topic_name_set)
            ros2_pub_topics, ros2_sub_topics = divide_ros2_topic_into_pubsub(overlapping_topics)

            # ======== Subscribers ========
            print("  Subscribers:")
            agnocast_sub_set = set(agnocast_subscribers)
            for sub in subscribers:
                if sub.name in agnocast_sub_set:
                    label = get_agnocast_label(sub.name, ros2_sub_topics, ros2_pub_topics)
                    print(f"    {sub.name}: {', '.join(sub.types)} {label}")
                else:
                    print(f"    {sub.name}: {', '.join(sub.types)}")

            ros2_sub_name_set = {sub.name for sub in subscribers}
            for agnocast_sub in agnocast_subscribers:
                if agnocast_sub in ros2_sub_name_set:
                    continue
                matching_topics = [topic for topic in ros2_topic_dir if topic['name'] == agnocast_sub]
                if matching_topics:
                    topic_types = '; '.join([', '.join(topic['types']) for topic in matching_topics])
                    print(f"    {agnocast_sub}: {topic_types} {get_agnocast_label(agnocast_sub, ros2_sub_topics, ros2_pub_topics)}")
                else:
                    type_label = gossip_topic_types.get(agnocast_sub, '<UNKNOWN>')
                    print(f"    {agnocast_sub}: {type_label} {get_agnocast_label(agnocast_sub, ros2_sub_topics, ros2_pub_topics)}")

            # ======== Publishers ========
            print("  Publishers:")
            agnocast_pub_set = set(agnocast_publishers)
            for pub in publishers:
                if pub.name in agnocast_pub_set:
                    label = get_agnocast_label(pub.name, ros2_sub_topics, ros2_pub_topics)
                    print(f"    {pub.name}: {', '.join(pub.types)} {label}")
                else:
                    print(f"    {pub.name}: {', '.join(pub.types)}")

            ros2_pub_name_set = {pub.name for pub in publishers}
            for agnocast_pub in agnocast_publishers:
                if agnocast_pub in ros2_pub_name_set:
                    continue
                matching_topics = [topic for topic in ros2_topic_dir if topic['name'] == agnocast_pub]
                if matching_topics:
                    topic_types = '; '.join([', '.join(topic['types']) for topic in matching_topics])
                    print(f"    {agnocast_pub}: {topic_types} {get_agnocast_label(agnocast_pub, ros2_sub_topics, ros2_pub_topics)}")
                else:
                    type_label = gossip_topic_types.get(agnocast_pub, '<UNKNOWN>')
                    print(f"    {agnocast_pub}: {type_label} {get_agnocast_label(agnocast_pub, ros2_sub_topics, ros2_pub_topics)}")

            # ======== Service ========
            print("  Service Servers:")
            for service in service_servers:
                print(f"    {service.name}: {', '.join(service.types)}")

            for service_name in agnocast_servers:
                print(f"    {service_name}: <UNKNOWN> {get_agnocast_label(service_name, ros2_sub_topics, ros2_pub_topics)}")

            print("  Service Clients:")
            for client in service_clients:
                print(f"    {client.name}: {', '.join(client.types)}")

            for service_name in agnocast_clients:
                print(f"    {service_name}: <UNKNOWN> {get_agnocast_label(service_name, ros2_sub_topics, ros2_pub_topics)}")

            # ======== Action ========
            print("  Action Servers:")
            for action in action_servers:
                print(f"    {action.name}: {', '.join(action.types)}")

            print("  Action Clients:")
            for action in action_clients:
                print(f"    {action.name}: {', '.join(action.types)}")
            ########################################################################
