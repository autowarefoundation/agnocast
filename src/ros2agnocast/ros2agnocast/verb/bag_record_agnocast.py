from ros2bag.verb.record import RecordVerb
from ros2agnocast._a2r_bridge_activator import A2rBridgeActivator


class BagRecordAgnocastVerb(RecordVerb):
    """Record ROS data to a bag, with automatic A2R bridge activation for Agnocast topics."""

    def main(self, *, args):
        # TODO: Pass a topic filter to A2rBridgeActivator so that bridges are only activated
        #       for the topics being recorded. The filter used by the ros2 bag record verb is as follows:
        #         -a / --all, --all-topics, [Topic ...] (positional), --topics, --topic-types,
        #         -e / --regex, --exclude-topics, --exclude-topic-types, --exclude-regex,
        #         --all-services, --services, --exclude-services
        # NOTE: Agnocast service is not-supported.
        with A2rBridgeActivator(log_level=args.log_level):
            return super().main(args=args)
