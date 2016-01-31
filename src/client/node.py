
import rospy
import sys
import time
import string
import publishermanager as pm
import json
from rospy_message_converter import message_converter as mc
from connection import Connection
from geometry_msgs.msg import Point


NODE_NAME = "jammi"

class JammiNode(object):

    def __init__(self, host, port, name, topics):
        self.host = host
        self.port = port
        self.name = name
        self.conn = Connection(host, port, name)
        self.conn.start()
        self.subs = dict()
        self.topics = topics
        self.pub_man = pm.PublisherManager()

    def run(self):
        for topic, msg_type in self.topics:
            self.create_subscriber(topic, msg_type)
        while not rospy.is_shutdown():
            updates = self.conn.updates()
            for v in updates.values():
                self.pub_man.publish(v)

    def create_subscriber(self, topic, msg_type):
        namespace, msg_name = msg_type.split("/")
        mod = __import__(namespace + ".msg")
        msg_cls = getattr(mod.msg, msg_name)
        cb = self.create_callback(topic, msg_type)
        self.subs[topic] = rospy.Subscriber(topic, msg_cls, cb)
        return self

    def create_callback(self, topic, msg_type):
        def callback(msg):
            data = dict()
            data["to"] = "*"
            data["from"] = self.name
            data["topic"] = "/{}{}".format(self.name, topic)
            data["type"] = msg_type
            data["stamp"] = time.time()
            data["msg"] = mc.convert_ros_message_to_dictionary(msg)
            payload = json.dumps(data)
            self.conn.send_message(payload)
        return callback


if __name__ == "__main__":
    rospy.init_node(sys.argv[1], anonymous=False)
    topics = [("/state", "geometry_msgs/Point")]
    jn = JammiNode("localhost", 9000, sys.argv[1], topics)
    jn.run()
