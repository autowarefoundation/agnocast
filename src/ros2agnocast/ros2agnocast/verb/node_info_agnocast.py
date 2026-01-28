import ctypes
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import (
    get_action_client_info, get_action_server_info, get_node_names,
    get_publisher_info, get_service_client_info, get_service_server_info, get_subscriber_info
)
from ros2topic.api import get_topic_names_and_types
from ros2node.verb import VerbExtension

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
        parser.add_argument(
            '--debug',
            '-d',
            action='store_true',
            help='Show additional debug information (e.g., whether topic is bridged)')

    def main(self, *, args):
        node_name = args.node_name

        with NodeStrategy(None) as node:
            node_names = [n.full_name for n in get_node_names(node=node)]
            if node_name not in node_names:
                print(f"Error: The node '{node_name}' does not exist.")
                return

            lib = ctypes.CDLL("libagnocast_ioctl_wrapper.so")
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

            def is_topic_bridged(topic_name):
                topic_name_bytes = topic_name.encode('utf-8')

                # Check Agnocast subscribers
                sub_count = ctypes.c_int()
                sub_array = lib.get_agnocast_sub_nodes(topic_name_bytes, ctypes.byref(sub_count))
                
                try:
                    if any(sub_array[i].is_bridge for i in range(sub_count.value)):
                        return True
                finally:
                    if sub_count.value > 0 and sub_array:
                        lib.free_agnocast_topic_info_ret(sub_array)

                # Check Agnocast publishers
                pub_count = ctypes.c_int()
                pub_array = lib.get_agnocast_pub_nodes(topic_name_bytes, ctypes.byref(pub_count))
                
                try:
                    if any(pub_array[i].is_bridge for i in range(pub_count.value)):
                        return True
                finally:
                    if pub_count.value > 0 and pub_array:
                        lib.free_agnocast_topic_info_ret(pub_array)

                return False

            def get_agnocast_label(topic_name):
                """Get the appropriate label for an Agnocast-enabled topic."""
                if args.debug and is_topic_bridged(topic_name):
                    return "(Agnocast enabled, bridged)"
                return "(Agnocast enabled)"

            node_name_bytes = args.node_name.encode('utf-8')

            # Topic names of the owned Agnocast subscribers
            agnocast_subscribers = []
            # Topic names of the owned Agnocast publishers
            agnocast_publishers = []
            # Service names of the owned Agnocast servers
            agnocast_servers = []
            # Service names of the owned Agnocast clients
            agnocast_clients = []

            sub_topic_count = ctypes.c_int()
            sub_topic_array = lib.get_agnocast_sub_topics(node_name_bytes, ctypes.byref(sub_topic_count))
            for i in range(sub_topic_count.value):
                topic_ptr = ctypes.cast(sub_topic_array[i], ctypes.c_char_p)
                topic_name = topic_ptr.value.decode('utf-8')

                service_name = service_name_from_request_topic(topic_name)
                if service_name is not None:
                    agnocast_servers.append(service_name)
                    continue

                service_name = service_name_from_response_topic(topic_name)
                if service_name is not None:
                    agnocast_clients.append(service_name)
                    continue

                agnocast_subscribers.append(topic_name)
            if sub_topic_count.value != 0:
                lib.free_agnocast_topics(sub_topic_array, sub_topic_count)

            pub_topic_count = ctypes.c_int()
            pub_topic_array = lib.get_agnocast_pub_topics(node_name_bytes, ctypes.byref(pub_topic_count))
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

                agnocast_publishers.append(topic_name)
            if pub_topic_count.value != 0:
                lib.free_agnocast_topics(pub_topic_array, pub_topic_count)


            ########################################################################
            # Print node info
            ########################################################################
            subscribers = get_subscriber_info(node=node, remote_node_name=node_name)
            publishers = get_publisher_info(node=node, remote_node_name=node_name)
            all_topics_raw = get_topic_names_and_types(node=node)
            all_topics = [{'name': topic_name, 'types': topic_types} for topic_name, topic_types in all_topics_raw]

            # ======== Subscribers ========
            print("  Subscribers:")
            for sub in subscribers:
                if sub.name in agnocast_subscribers:
                    print(f"    {sub.name}: {', '.join(sub.types)} {get_agnocast_label(sub.name)}")
                else:
                    print(f"    {sub.name}: {', '.join(sub.types)}")

            for agnocast_sub in agnocast_subscribers:
                if agnocast_sub in [sub.name for sub in subscribers]:
                    continue
                matching_topics = [topic for topic in all_topics if topic['name'] == agnocast_sub]
                if matching_topics:
                    topic_types = '; '.join([', '.join(topic['types']) for topic in matching_topics])
                    print(f"    {agnocast_sub}: {topic_types} {get_agnocast_label(agnocast_sub)}")
                else:
                    print(f"    {agnocast_sub}: <UNKNOWN> {get_agnocast_label(agnocast_sub)}")

            # ======== Publishers ========
            print("  Publishers:")
            for pub in publishers:
                if pub.name in agnocast_publishers:
                    print(f"    {pub.name}: {', '.join(pub.types)} {get_agnocast_label(pub.name)}")
                else:
                    print(f"    {pub.name}: {', '.join(pub.types)}")

            for agnocast_pub in agnocast_publishers:
                if agnocast_pub in [pub.name for pub in publishers]:
                    continue
                matching_topics = [topic for topic in all_topics if topic['name'] == agnocast_pub]
                if matching_topics:
                    topic_types = '; '.join([', '.join(topic['types']) for topic in matching_topics])
                    print(f"    {agnocast_pub}: {topic_types} {get_agnocast_label(agnocast_pub)}")
                else:
                    print(f"    {agnocast_pub}: <UNKNOWN> {get_agnocast_label(agnocast_pub)}")

            # ======== Service ========
            print("  Service Servers:")
            service_servers = get_service_server_info(node=node, remote_node_name=node_name)
            for service in service_servers:
                print(f"    {service.name}: {', '.join(service.types)}")

            for service_name in agnocast_servers:
                print(f"    {service_name}: <UNKNOWN> {get_agnocast_label(service_name)}")

            print("  Service Clients:")
            service_clients = get_service_client_info(node=node, remote_node_name=node_name)
            for client in service_clients:
                print(f"    {client.name}: {', '.join(client.types)}")

            for service_name in agnocast_clients:
                print(f"    {service_name}: <UNKNOWN> {get_agnocast_label(service_name)}")

            # ======== Action ========
            print("  Action Servers:")
            action_servers = get_action_server_info(node=node, remote_node_name=node_name)
            for action in action_servers:
                print(f"    {action.name}: {', '.join(action.types)}")

            print("  Action Clients:")
            action_clients = get_action_client_info(node=node, remote_node_name=node_name)
            for action in action_clients:
                print(f"    {action.name}: {', '.join(action.types)}")
            ########################################################################
