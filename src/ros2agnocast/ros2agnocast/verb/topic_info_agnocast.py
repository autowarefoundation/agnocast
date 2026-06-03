from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import TopicNameCompleter
from ros2node.verb import VerbExtension

from ros2agnocast.discovery import (
    add_gossip_timeout_arg,
    collect_announcements_with_fallback,
    topic_endpoints,
    warn_if_gossip_timeout_overridden,
    warn_if_using_fallback,
)

class TopicInfoAgnocastVerb(VerbExtension):
    """Print information about a topic including Agnocast."""

    def add_arguments(self, parser, cli_name):
        add_strategy_node_arguments(parser)
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to get info (e.g. '/chatter') including Agnocast.")
        parser.add_argument(
            '--verbose',
            '-v',
            action='store_true',
            help='Prints detailed information like the node name, node namespace, topic type, '
                 'topic type hash, GUID, and QoS Profile of the publishers and subscribers to '
                 'this topic. In case of Agnocast, only the supported QoS parameters are '
                 'displayed')
        parser.add_argument(
            '--debug',
            '-d',
            action='store_true',
            help='Include internal bridge nodes (agnocast_bridge_node_*) in the output')
        add_gossip_timeout_arg(parser)
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')

    def split_full_node_name(self, full_node_name):
        full_node_name = full_node_name.rstrip("/") if full_node_name != "/" else full_node_name
        namespace, _, node_name = full_node_name.rpartition("/")
        namespace = namespace if namespace else "/"
        return namespace, node_name

    def print_publishers_info(self, ros2_pub_infos, topic_types, pub_topic_info_rets, args, line_end):
        # Filter out bridge nodes unless debug mode
        if not args.debug:
            pub_topic_info_rets = [p for p in pub_topic_info_rets if not p['is_bridge']]
        agnocast_pub_count = len(pub_topic_info_rets)
        ros2_pub_count = len(ros2_pub_infos)

        print('ROS 2 Publisher count: %d' % ros2_pub_count)
        print('Agnocast Publisher count: %d' % agnocast_pub_count, end=line_end)

        if args.verbose:
            try:
                for info in ros2_pub_infos:
                    print('Node name: %s' % info.node_name)
                    print('Node namespace: %s' % info.node_namespace)
                    print('Topic type: %s' % info.topic_type)
                    print('Endpoint type: %s' % info.endpoint_type.name)
                    print('GID: %s' % '.'.join(format(b, '02x') for b in info.endpoint_gid))
                    print('QoS profile:')
                    print('  Reliability: %s' % info.qos_profile.reliability.name)
                    print('  History (Depth): %s (%d)' % (info.qos_profile.history.name, info.qos_profile.depth))
                    print('  Durability: %s' % info.qos_profile.durability.name)
                    print('  Lifespan: %s' % info.qos_profile.lifespan)
                    print('  Deadline: %s' % info.qos_profile.deadline)
                    print('  Liveliness: %s' % info.qos_profile.liveliness.name)
                    print('  Liveliness lease duration: %s' % info.qos_profile.liveliness_lease_duration, end=line_end)

                for info in pub_topic_info_rets:
                    nodespace, node_name = self.split_full_node_name(info['node_name'])
                    print('Node name: %s' % node_name)
                    print('Node namespace: %s' % nodespace)
                    print('Topic type: %s' % topic_types)
                    print('Endpoint type: PUBLISHER (Agnocast enabled)')
                    print('QoS profile:')
                    print('  History (Depth): KEEP_LAST (%d)' % info['qos_depth'])
                    if info['qos_is_transient_local']:
                        print('  Durability: TRANSIENT_LOCAL', end=line_end)
                    else:
                        print('  Durability: VOLATILE', end=line_end)
            except NotImplementedError as e:
                return str(e)

    def print_subscribers_info(self, ros2_sub_infos, topic_types, sub_topic_info_rets, args, line_end):
        # Filter out bridge nodes unless debug mode
        if not args.debug:
            sub_topic_info_rets = [s for s in sub_topic_info_rets if not s['is_bridge']]
        agnocast_sub_count = len(sub_topic_info_rets)
        ros2_sub_count = len(ros2_sub_infos)

        print('ROS 2 Subscription count: %d' % ros2_sub_count)
        print('Agnocast Subscription count: %d' % agnocast_sub_count, end=line_end)

        if args.verbose:
            try:
                for info in ros2_sub_infos:
                    print('Node name: %s' % info.node_name)
                    print('Node namespace: %s' % info.node_namespace)
                    print('Topic type: %s' % info.topic_type)
                    print('Endpoint type: %s' % info.endpoint_type.name)
                    print('GID: %s' % '.'.join(format(b, '02x') for b in info.endpoint_gid))
                    print('QoS profile:')
                    print('  Reliability: %s' % info.qos_profile.reliability.name)
                    print('  History (Depth): %s (%d)' % (info.qos_profile.history.name, info.qos_profile.depth))
                    print('  Durability: %s' % info.qos_profile.durability.name)
                    print('  Lifespan: %s' % info.qos_profile.lifespan)
                    print('  Deadline: %s' % info.qos_profile.deadline)
                    print('  Liveliness: %s' % info.qos_profile.liveliness.name)
                    print('  Liveliness lease duration: %s' % info.qos_profile.liveliness_lease_duration, end=line_end)

                for info in sub_topic_info_rets:
                    nodespace, node_name = self.split_full_node_name(info['node_name'])
                    print('Node name: %s' % node_name)
                    print('Node namespace: %s' % nodespace)
                    print('Topic type: %s' % topic_types)
                    print('Endpoint type: SUBSCRIPTION (Agnocast enabled)')
                    print('QoS profile:')
                    print('  History (Depth): KEEP_LAST (%d)' % info['qos_depth'])
                    if info['qos_is_transient_local']:
                        print('  Durability: TRANSIENT_LOCAL', end=line_end)
                    else:
                        print('  Durability: VOLATILE', end=line_end)
            except NotImplementedError as e:
                return str(e)

    def main(self, *, args):
        warn_if_gossip_timeout_overridden(args)
        with NodeStrategy(None) as node:
            topic_name = args.topic_name

            snapshots, used_fallback = collect_announcements_with_fallback(
                node, timeout_sec=args.gossip_timeout)
            warn_if_using_fallback(node, used_fallback, args.gossip_timeout)
            pub_endpoints, sub_endpoints = topic_endpoints(snapshots, topic_name)

            def to_info_ret(ep):
                return {
                    "node_name": ep.node_name,
                    "qos_depth": ep.qos_depth,
                    "qos_is_transient_local": ep.qos_is_transient_local,
                    "is_bridge": ep.is_bridge,
                }
            sub_topic_info_rets = [to_info_ret(ep) for ep in sub_endpoints]
            pub_topic_info_rets = [to_info_ret(ep) for ep in pub_endpoints]

            # Resolve type from gossip when DDS does not provide one.
            gossip_type_name = next(
                (topic.type_name for snap in snapshots
                 for topic in snap.topics
                 if topic.topic_name == topic_name and topic.type_name),
                '')

            # get bridge node names
            bridge_node_names = set()
            for info in sub_topic_info_rets + pub_topic_info_rets:
                if info['is_bridge']:
                    _, name = self.split_full_node_name(info['node_name'])
                    bridge_node_names.add(name)

            # get ROS 2 pub/sub info for this topic
            ros2_pub_infos_all = []
            ros2_sub_infos_all = []
            try:
                ros2_pub_infos_all = list(node.get_publishers_info_by_topic(topic_name))
            except NotImplementedError:
                pass
            try:
                ros2_sub_infos_all = list(node.get_subscriptions_info_by_topic(topic_name))
            except NotImplementedError:
                pass

            topic_types = []
            for info in ros2_pub_infos_all + ros2_sub_infos_all:
                if info.topic_type and info.topic_type not in topic_types:
                    topic_types.append(info.topic_type)

            if args.debug:
                ros2_pub_infos = ros2_pub_infos_all
                ros2_sub_infos = ros2_sub_infos_all
            else:
                ros2_pub_infos = [i for i in ros2_pub_infos_all if i.node_name not in bridge_node_names]
                ros2_sub_infos = [i for i in ros2_sub_infos_all if i.node_name not in bridge_node_names]

            # check if topic exists
            if not topic_types:
                if not pub_topic_info_rets and not sub_topic_info_rets:
                    return 'Unknown topic: %s' % topic_name
                else:
                    topic_types = [gossip_type_name] if gossip_type_name else ['<UNKNOWN>']

            ########################################################################
            # print topic info
            ########################################################################
            line_end = '\n'
            if args.verbose:
                line_end = '\n\n'
            type_str = topic_types[0] if len(topic_types) == 1 else topic_types
            print('Type: %s' % type_str, end=line_end)

            print_publishers_info_ret = self.print_publishers_info(ros2_pub_infos, type_str, pub_topic_info_rets, args, line_end)
            if print_publishers_info_ret:
                return print_publishers_info_ret
            print_subscribers_info_ret = self.print_subscribers_info(ros2_sub_infos, type_str, sub_topic_info_rets, args, line_end)
            if print_subscribers_info_ret:
                return print_subscribers_info_ret
            ########################################################################
