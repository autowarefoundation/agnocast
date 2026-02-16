import ctypes
from enum import Enum
from contextlib import contextmanager
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import (
    get_action_client_info, get_action_server_info, get_node_names,
    get_publisher_info, get_service_client_info, get_service_server_info, get_subscriber_info
)
from ros2topic.api import get_topic_names_and_types
from ros2node.verb import VerbExtension

class BridgeStatus(Enum):
    NONE = 0
    ROS2_TO_AGNOCAST = 1
    AGNOCAST_TO_ROS2= 2
    BIDIRECTION = 3

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

    def main(self, *, args):
        node_name = args.node_name


        with NodeStrategy(None) as node:
            lib = ctypes.CDLL("libagnocast_ioctl_wrapper.so")
            lib.get_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
            lib.free_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.POINTER(ctypes.c_char)), ctypes.c_int]
            lib.free_agnocast_topics.restype = None
            lib.get_agnocast_sub_topics.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_sub_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
            lib.get_agnocast_pub_topics.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_pub_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
            lib.free_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.POINTER(ctypes.c_char)), ctypes.c_int]
            lib.free_agnocast_topics.restype = None

            # For bridge detection, we need to get nodes by topic
            lib.get_agnocast_sub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_sub_nodes.restype = ctypes.POINTER(TopicInfoRet)
            lib.get_agnocast_pub_nodes.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_pub_nodes.restype = ctypes.POINTER(TopicInfoRet)
            lib.free_agnocast_topic_info_ret.argtypes = [ctypes.POINTER(TopicInfoRet)]
            lib.free_agnocast_topic_info_ret.restype = None

            @contextmanager
            def agnocast_info_array(lib_func, topic_name_bytes):
                count = ctypes.c_int()
                array = lib_func(topic_name_bytes, ctypes.byref(count))
                try:
                    yield array[:count.value] if array else []
                finally:
                    if array:
                        lib.free_agnocast_topic_info_ret(array)

            def get_bridge_status(topic_name): 
                name_b = topic_name.encode('utf-8')

                has_sub_bridge = False
                has_pub_bridge = False

                with agnocast_info_array(lib.get_agnocast_sub_nodes, name_b) as nodes:
                    has_sub_bridge = any(n.is_bridge for n in nodes)
                with agnocast_info_array(lib.get_agnocast_pub_nodes, name_b) as nodes:
                    has_pub_bridge = any(n.is_bridge for n in nodes)

                mapping = {
                    (True, True):   BridgeStatus.BIDIRECTION,
                    (True, False):  BridgeStatus.AGNOCAST_TO_ROS2,
                    (False, True):  BridgeStatus.ROS2_TO_AGNOCAST,
                    (False, False): BridgeStatus.NONE,
                }
                
                return mapping[(has_sub_bridge, has_pub_bridge)]

            def get_agnocast_label(topic_name, ros2_sub_topics, ros2_pub_topics):
                """Get the appropriate label for an Agnocast-enabled topic."""

                suffix = "(Agnocast enabled)"
                if topic_name not in ros2_sub_topics and topic_name not in ros2_pub_topics:
                    return suffix  # No bridge info if only one endpoint exists

                match get_bridge_status(topic_name):
                    case BridgeStatus.BIDIRECTION:
                        suffix = "(Agnocast enabled, bridged)"
                    case BridgeStatus.ROS2_TO_AGNOCAST:
                        if topic_name in ros2_pub_topics:
                            suffix = "(Agnocast enabled, bridged)"
                    case BridgeStatus.AGNOCAST_TO_ROS2:
                        if topic_name in ros2_sub_topics:
                            suffix = "(Agnocast enabled, bridged)"
                    case BridgeStatus.NONE:
                        suffix = "(WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"
                return suffix
            
            def get_agnocast_node_info(topic_list, node_name):
                sub_topic_set = set()
                pub_topic_set = set()
                server_set = set()
                client_set = set()

                for topic_name in topic_list:
                    topic_name_bytes = topic_name.encode('utf-8')
                    # Check Agnocast subscribers
                    sub_count = ctypes.c_int()
                    sub_array = lib.get_agnocast_sub_nodes(topic_name_bytes, ctypes.byref(sub_count))
                    if sub_array:
                        try:
                            for i in range(sub_count.value):
                                if sub_array[i].node_name.decode('utf-8') == node_name:
                                    # Check if this topic is used by a service server
                                    service_name = service_name_from_request_topic(topic_name)
                                    if service_name is not None:
                                        server_set.add(service_name)
                                        break
                                    service_name = service_name_from_response_topic(topic_name)
                                    if service_name is not None:
                                        client_set.add(service_name)
                                        break
                                    sub_topic_set.add(topic_name)
                        finally:
                            lib.free_agnocast_topic_info_ret(sub_array)

                    # Check Agnocast publishers
                    pub_count = ctypes.c_int()
                    pub_array = lib.get_agnocast_pub_nodes(topic_name_bytes, ctypes.byref(pub_count))
                    if pub_array:
                        try:
                            for i in range(pub_count.value):
                                if pub_array[i].node_name.decode('utf-8') == node_name:
                                    # Skip topic names used by services.
                                    # They have already been accounted for during the subscription topic scan.
                                    if (
                                        service_name_from_request_topic(topic_name) is not None
                                        or service_name_from_response_topic(topic_name) is not None
                                    ):
                                        continue
                                    pub_topic_set.add(topic_name)
                        finally:
                            lib.free_agnocast_topic_info_ret(pub_array)

                return list(sub_topic_set), list(pub_topic_set), list(server_set), list(client_set)
            
            def get_ros2_node_agnocast_topic(node_name):
                sub_topic_list = []
                pub_topic_list = []
                server_list = []
                client_list = []

                sub_topic_count = ctypes.c_int()
                sub_topic_array = lib.get_agnocast_sub_topics(node_name, ctypes.byref(sub_topic_count))
                for i in range(sub_topic_count.value):
                    topic_ptr = ctypes.cast(sub_topic_array[i], ctypes.c_char_p)
                    topic_name = topic_ptr.value.decode('utf-8')

                    service_name = service_name_from_request_topic(topic_name)
                    if service_name is not None:
                        server_list.append(service_name)
                        continue

                    service_name = service_name_from_response_topic(topic_name)
                    if service_name is not None:
                        client_list.append(service_name)
                        continue

                    sub_topic_list.append(topic_name)
                if sub_topic_count.value != 0:
                    lib.free_agnocast_topics(sub_topic_array, sub_topic_count)

                pub_topic_count = ctypes.c_int()
                pub_topic_array = lib.get_agnocast_pub_topics(node_name, ctypes.byref(pub_topic_count))
                for i in range(pub_topic_count.value):
                    topic_ptr = ctypes.cast(pub_topic_array[i], ctypes.c_char_p)
                    topic_name = topic_ptr.value.decode('utf-8')

                    # Skip topic names used by services.
                    # They have already been accounted for during the subscription topic scan.
                    if (
                        service_name_from_request_topic(topic_name) is not None
                        or service_name_from_response_topic(topic_name) is not None
                    ):
                        continue

                    pub_topic_list.append(topic_name)
                if pub_topic_count.value != 0:
                    lib.free_agnocast_topics(pub_topic_array, pub_topic_count)
                
                return sub_topic_list, pub_topic_list, server_list, client_list

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

            # Topic names of the owned Agnocast subscribers
            agnocast_subscribers = []
            # Topic names of the owned Agnocast publishers
            agnocast_publishers = []
            # Service names of the owned Agnocast servers
            agnocast_servers = []
            # Service names of the owned Agnocast clients
            agnocast_clients = []

            # Get Agnocast all topics
            topic_count = ctypes.c_int()
            agnocast_topic_array = lib.get_agnocast_topics(ctypes.byref(topic_count))
            agnocast_topics = []
            for i in range(topic_count.value):
                topic_ptr = ctypes.cast(agnocast_topic_array[i], ctypes.c_char_p)
                topic_name = topic_ptr.value.decode('utf-8')
                agnocast_topics.append(topic_name)
            if topic_count.value != 0:
                lib.free_agnocast_topics(agnocast_topic_array, topic_count)

            # Get Agnocast node info
            agnocast_subscribers, agnocast_publishers, agnocast_servers, agnocast_clients = get_agnocast_node_info(agnocast_topics, node_name)

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
                node_name_bytes = node_name.encode('utf-8')
                agnocast_subscribers, agnocast_publishers, agnocast_servers, agnocast_clients = get_ros2_node_agnocast_topic(node_name_bytes)
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
            ros2_topic_name_list = [topic['name'] for topic in ros2_topic_dir]
            ros2_pub_topics, ros2_sub_topics = divide_ros2_topic_into_pubsub(ros2_topic_name_list)

            # ======== Subscribers ========
            print("  Subscribers:")
            for sub in subscribers:
                print(f"    {sub.name}: {', '.join(sub.types)}")

            for agnocast_sub in agnocast_subscribers:
                if agnocast_sub in [sub.name for sub in subscribers]:
                    continue
                matching_topics = [topic for topic in ros2_topic_dir if topic['name'] == agnocast_sub]
                if matching_topics:
                    topic_types = '; '.join([', '.join(topic['types']) for topic in matching_topics])
                    print(f"    {agnocast_sub}: {topic_types} {get_agnocast_label(agnocast_sub, ros2_sub_topics, ros2_pub_topics)}")
                else:
                    print(f"    {agnocast_sub}: <UNKNOWN> {get_agnocast_label(agnocast_sub, ros2_sub_topics, ros2_pub_topics)}")

            # ======== Publishers ========
            print("  Publishers:")
            for pub in publishers:
                print(f"    {pub.name}: {', '.join(pub.types)}")

            for agnocast_pub in agnocast_publishers:
                if agnocast_pub in [pub.name for pub in publishers]:
                    continue
                matching_topics = [topic for topic in ros2_topic_dir if topic['name'] == agnocast_pub]
                if matching_topics:
                    topic_types = '; '.join([', '.join(topic['types']) for topic in matching_topics])
                    print(f"    {agnocast_pub}: {topic_types} {get_agnocast_label(agnocast_pub, ros2_sub_topics, ros2_pub_topics)}")
                else:
                    print(f"    {agnocast_pub}: <UNKNOWN> {get_agnocast_label(agnocast_pub, ros2_sub_topics, ros2_pub_topics)}")

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
