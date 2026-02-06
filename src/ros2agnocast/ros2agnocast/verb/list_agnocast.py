import ctypes
from enum import Enum
from contextlib import contextmanager
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension

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
class ListAgnocastVerb(VerbExtension):
    "Output a list of available topics including Agnocast"

    def main(self, *, args):
        with NodeStrategy(None) as node:
            lib = ctypes.CDLL("libagnocast_ioctl_wrapper.so")
            lib.get_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
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

            agnocast_topics = remove_service_topic(agnocast_topics)
            
            # Get ros2 topics
            ros2_topics_data = get_topic_names_and_types(node=node)
            ros2_topics = [name for name, _ in ros2_topics_data]
            ros2_pub_topics, ros2_sub_topics = divide_ros2_topic_into_pubsub(ros2_topics)

            ########################################################################
            # Print topic list
            ########################################################################
            agnocast_topics_set = set(agnocast_topics)
            ros2_topics_set = set(ros2_pub_topics) | set(ros2_sub_topics)

            for topic in sorted(agnocast_topics_set | ros2_topics_set):
                if topic in agnocast_topics_set and topic not in ros2_topics_set:
                    suffix = " (Agnocast enabled)"
                elif topic in ros2_topics_set and topic not in agnocast_topics_set:
                    suffix = ""
                else:
                    match get_bridge_status(topic):
                        case BridgeStatus.BIDIRECTION:
                            suffix = " (Agnocast enabled, bridged)"
                        case BridgeStatus.ROS2_TO_AGNOCAST:
                            if topic in ros2_pub_topics:
                                suffix = " (Agnocast enabled, bridged)"
                            else:
                                suffix = " (Agnocast enabled)"
                        case BridgeStatus.AGNOCAST_TO_ROS2:
                            if topic in ros2_sub_topics:
                                suffix = " (Agnocast enabled, bridged)"
                            else:
                                suffix = " (Agnocast enabled)"
                        case BridgeStatus.NONE:
                            suffix = " (WARN: Agnocast and ROS2 endpoints exist but bridge is not active)"
                print(f"{topic}{suffix}")
