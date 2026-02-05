import ctypes
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension

class ListAgnocastVerb(VerbExtension):
    "Output a list of available topics including Agnocast"

    def main(self, *, args):
        with NodeStrategy(None) as node:
            lib = ctypes.CDLL("libagnocast_ioctl_wrapper.so")
            lib.get_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.c_int)]
            lib.get_agnocast_topics.restype = ctypes.POINTER(ctypes.POINTER(ctypes.c_char))
            lib.free_agnocast_topics.argtypes = [ctypes.POINTER(ctypes.POINTER(ctypes.c_char)), ctypes.c_int]
            lib.free_agnocast_topics.restype = None
            
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
            # Get ros2 topics
            ros2_topics_data = get_topic_names_and_types(node=node)
            ros2_topics = [name for name, _ in ros2_topics_data]

            ########################################################################
            # Print topic list
            ########################################################################
            agnocast_topics_set = set(agnocast_topics)
            ros2_topics_set = set(ros2_topics)

            for topic in sorted(agnocast_topics_set | ros2_topics_set):
                suffix = " (Agnocast enabled)" if topic in agnocast_topics_set else ""
                print(f"{topic}{suffix}")

