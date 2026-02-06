import ctypes
import sys

from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2topic.verb import VerbExtension

class TopicInfoRet(ctypes.Structure):
    _fields_ = [
        ("node_name", ctypes.c_char * 256),
        ("qos_depth", ctypes.c_uint32),
        ("qos_is_transient_local", ctypes.c_bool),
        # Agnocast does not natively support reliability configuration,
        # but this field is required to pass the QoS profile to the ROS 2 bridge.
        ("qos_is_reliable", ctypes.c_bool),
        ("is_bridge", ctypes.c_bool),
    ]
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

    def main(self, *, args):
        with NodeStrategy(None) as node:
            lib = ctypes.CDLL("libagnocast_ioctl_wrapper.so")
            lib.get_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
            lib.free_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.POINTER(ctypes.c_char)), ctypes.c_int]
            lib.free_agnocast_topics.restype = None

            lib.get_agnocast_sub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_sub_nodes.restype = ctypes.POINTER(TopicInfoRet)
            lib.get_agnocast_pub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_pub_nodes.restype = ctypes.POINTER(TopicInfoRet)
            lib.free_agnocast_topic_info_ret.argtypes = [ctypes.POINTER(TopicInfoRet)]
            lib.free_agnocast_topic_info_ret.restype = None

            def get_node_name_set(topic_name):
                topic_name_bytes = topic_name.encode('utf-8')
                node_names = set()

                # Check Agnocast subscribers
                sub_count = ctypes.c_int()
                sub_array = lib.get_agnocast_sub_nodes(topic_name_bytes, ctypes.byref(sub_count))
                if sub_array:
                    for i in range(sub_count.value):
                        node_names.add(sub_array[i].node_name.decode('utf-8'))
                    lib.free_agnocast_topic_info_ret(sub_array)

                # Check Agnocast publishers
                pub_count = ctypes.c_int()
                pub_array = lib.get_agnocast_pub_nodes(topic_name_bytes, ctypes.byref(pub_count))
                if pub_array:
                    for i in range(pub_count.value):
                        node_names.add(pub_array[i].node_name.decode('utf-8'))
                    lib.free_agnocast_topic_info_ret(pub_array)

                return node_names
            
            # Get Agnocast topics
            topic_count = ctypes.c_int()
            agnocast_topic_array = lib.get_agnocast_topics(ctypes.byref(topic_count))
            agnocast_topics = []
            for i in range(topic_count.value):
                topic_ptr = ctypes.cast(agnocast_topic_array[i], ctypes.c_char_p)
                topic_name = topic_ptr.value.decode('utf-8')
                agnocast_topics.append(topic_name)
            if topic_count.value != 0:
                lib.free_agnocast_topics(agnocast_topic_array, topic_count)

            # Get node names which contains Agnocast topics
            agnocast_node_name = set()
            for topic in agnocast_topics:
                agnocast_node_name = agnocast_node_name | get_node_name_set(topic)
            
            # Get ros2 node names
            ros2_node_name_list = get_node_names(node=node, include_hidden_nodes=args.all)
            ros2_node_name = {n.full_name for n in ros2_node_name_list}

            ########################################################################
            # Print node list
            ########################################################################
            merged_node_name = agnocast_node_name | ros2_node_name
            if not args.all and not args.debug:
                merged_node_name = {node for node in merged_node_name if not node.startswith("/agnocast_bridge_node_")}
            if args.count_nodes:
                total_nodes = len(agnocast_node_name | ros2_node_name)
                print(total_nodes)
            else:

                for node_name in sorted(merged_node_name):
                    if node_name in agnocast_node_name and node_name not in ros2_node_name:
                        suffix = " (Agnocast enabled)"
                    else:
                        suffix = ""
                    print(f"{node_name}{suffix}")
