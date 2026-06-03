from ros2topic.verb.delay import DelayVerb
from ros2agnocast.verb._bridged_ros2cli import add_bridge_arguments
from ros2agnocast.verb._bridged_ros2cli import spawn_bridge_and_run


class TopicDelayAgnocastVerb(DelayVerb):

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        add_bridge_arguments(parser)

    def main(self, *, args):
        return spawn_bridge_and_run(
            args, topic_name=args.topic, action_fn=lambda a: DelayVerb.main(self, args=a))
