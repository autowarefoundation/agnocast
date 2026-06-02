from ros2topic.verb.echo import EchoVerb
from ros2agnocast.verb._bridged_ros2cli import add_bridge_arguments
from ros2agnocast.verb._bridged_ros2cli import spawn_bridge_and_run


class TopicEchoAgnocastVerb(EchoVerb):

    def add_arguments(self, parser, cli_name):
        super().add_arguments(parser, cli_name)
        add_bridge_arguments(parser)

    def main(self, *, args):
        return spawn_bridge_and_run(
            args, topic_name=args.topic_name, action_fn=lambda a: EchoVerb.main(self, args=a),
            message_type=args.message_type)
