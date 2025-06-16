#!/usr/bin/env python3
import rospy
import yaml
from roslib.message import get_message_class
from rospkg import RosPack
import os

class TopicRelay:
    def __init__(self, config_path):
        self.relay_list = []
        self.load_config(config_path)

    def load_config(self, config_path):
        rospy.loginfo("Loading relay configuration from: %s", config_path)

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr("Failed to load YAML file: %s", str(e))
            return

        for entry in config.get('relay_topics', []):
            in_topic = entry.get('input')
            out_topic = entry.get('output')
            msg_type_str = entry.get('type')
            latch = entry.get('latch')
            if not in_topic or not msg_type_str:
                rospy.logwarn("Skipping invalid entry (missing input/type): %s", entry)
                continue

            if not latch:
                latch = False

            if not out_topic:
                out_topic = "/relay" + in_topic
                rospy.loginfo("Output topic not provided, using default: %s", out_topic)

            msg_class = get_message_class(msg_type_str)
            if msg_class is None:
                rospy.logerr("Invalid message type: %s", msg_type_str)
                continue

            try:
                pub = rospy.Publisher(out_topic, msg_class, queue_size=1, latch=latch)
                sub = rospy.Subscriber(
                    in_topic,
                    msg_class,
                    self.make_callback(pub),
                    queue_size=1,
                    tcp_nodelay=True
                )
                self.relay_list.append((sub, pub))
                rospy.loginfo("🔁 Relaying: %s → %s [%s]", in_topic, out_topic, msg_type_str)
            except Exception as e:
                rospy.logerr("Failed to set up relay for %s → %s: %s", in_topic, out_topic, str(e))

    def make_callback(self, publisher):
        return lambda msg: publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('multi_topic_relay')
    config_path = os.path.join(RosPack().get_path('stack_master'), 'config', 'relay_topics.yaml')
    # config_path = rospy.get_param('~config', 'relay_topics.yaml')
    TopicRelay(config_path)
    rospy.spin()
