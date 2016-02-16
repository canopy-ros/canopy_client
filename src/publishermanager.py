# Defines PublisherManager class.

import rospy
from rospy_message_converter import message_converter

# Manages ROS publishers for received messages.
class PublisherManager(object):

    def __init__(self):
        self.pubs = dict()

    # Converts message dictionary to ROS message for publishing.
    def create_msg(self, msg_dict, msg_type):
        namespace, msg_name = msg_type.split("/")
        mod = __import__(namespace + ".msg")
        msg_cls = getattr(mod.msg, msg_name)
        msg = message_converter.convert_dictionary_to_ros_message(
            msg_type, msg_dict)
        return msg, msg_cls

    # Publishes newly received messages.
    # Creates new Publishers for new topics.
    def publish(self, data):
        msg, msg_cls = self.create_msg(data["Msg"], data["Type"])
        topic = data["Topic"]
        if not topic in self.pubs.keys():
            self.pubs[topic] = rospy.Publisher(topic, msg_cls, queue_size=2)
        self.pubs[topic].publish(msg)
