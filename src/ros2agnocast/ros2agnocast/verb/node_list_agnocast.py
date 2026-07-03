from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2topic.verb import VerbExtension

from ros2agnocast.discovery import (
    add_gossip_timeout_arg,
    all_nodes,
    collect_announcements_with_fallback,
    warn_if_gossip_timeout_overridden,
    warn_if_using_fallback,
)

def split_fqn(fqn):
    namespace, _, name = fqn.rpartition('/')
    return (namespace or '/'), name

class ListAgnocastVerb(VerbExtension):
    "Output a list of available nodes including Agnocast::Node"

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            '-a', '--all', action='store_true',
            help='Display all nodes even hidden ones')
        parser.add_argument(
            '-c', '--count-nodes', action='store_true',
            help='Only display the number of nodes discovered')
        parser.add_argument(
            '-d', '--debug', action='store_true',
            help='Include internal bridge nodes (agnocast_bridge_node_*) in the output')
        add_gossip_timeout_arg(parser)

    def main(self, *, args):
        warn_if_gossip_timeout_overridden(args)
        with NodeStrategy(None) as node:
            snapshots, used_fallback = collect_announcements_with_fallback(
                node, timeout_sec=args.gossip_timeout)
            warn_if_using_fallback(node, used_fallback, args.gossip_timeout, snapshots)

            # Get node names which contains Agnocast topics from gossip.
            agnocast_node_name = all_nodes(snapshots)

            # TODO(bdm-k): The current impl determines shadow nodes in a heuristic way. We need to
            # invent a deterministic way to identify shadow nodes.

            def likely_shadow_node(fqn):
                if fqn not in agnocast_node_name:
                    return False

                ns, name = split_fqn(fqn)
                try:
                    # A normal rclcpp node is likely to have parameter services, so start by testing
                    # services.
                    if (
                        len(node.get_service_names_and_types_by_node(name, ns)) != 0
                        or len(node.get_publisher_names_and_types_by_node(name, ns)) != 0
                        or len(node.get_subscriber_names_and_types_by_node(name, ns)) != 0
                        or len(node.get_client_names_and_types_by_node(name, ns)) != 0
                    ):
                        return False
                    return True
                except Exception:
                    return False

            # Get ros2 node names.
            ros2_node_name_list = get_node_names(node=node, include_hidden_nodes=args.all)
            # Exclude shadow nodes so that the corresponding Agnocast nodes are listed with "(Agnocast enabled)"
            ros2_node_name = {n.full_name for n in ros2_node_name_list if not likely_shadow_node(n.full_name)}

            ########################################################################
            # Print node list
            ########################################################################
            merged_node_name = agnocast_node_name | ros2_node_name
            if not args.all and not args.debug:
                merged_node_name = {node for node in merged_node_name if not node.startswith("/agnocast_bridge_node_")}
            if args.count_nodes:
                total_nodes = len(merged_node_name)
                print(total_nodes)
            else:
                for node_name in sorted(merged_node_name):
                    if node_name in agnocast_node_name and node_name not in ros2_node_name:
                        suffix = " (Agnocast enabled)"
                    else:
                        suffix = ""
                    print(f"{node_name}{suffix}")
