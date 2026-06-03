from ros2bag.verb.record import RecordVerb
from ros2agnocast.verb._a2r_bridge_activator import A2rBridgeActivator


class BagRecordAgnocastVerb(RecordVerb):
    """Record ROS data to a bag, with automatic A2R bridge activation for Agnocast topics."""

    def main(self, *, args):
        with A2rBridgeActivator(log_level=args.log_level):
            return RecordVerb.main(self, args=args)
