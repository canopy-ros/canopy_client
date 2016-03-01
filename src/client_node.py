#!/usr/bin/env python
# Defines the ROSCloudNode class.

import rospy
import time
import publishermanager as pm
from rospy_message_converter import message_converter as mc
from connection import Connection
import threading


NODE_NAME = "roscloud_client"

# The ROS node object for the ROSCloud client.
# Manages all connections and subscribing.
# One instance per client node.
class ROSCloudNode(object):

    def __init__(self, host, port, name, broadcasting, private_key, description):
        self.host = host
        self.port = port
        self.name = name.replace(" ", "").replace("/", "")
        self.conn = dict()
        self.receiver = None
        self.descriptionConn = None
        self.subs = dict()
        self.broadcasting = broadcasting
        self.private_key = private_key
        self.description = description
        self.pub_man = pm.PublisherManager()
        self.timer = threading.Timer(0.1, self.descriptionSend)

    # Creates all connections and subscribers and starts them.
    # Runs a loop that checks for received messages.
    def run(self):
        for topic, msg_type, trusted in self.broadcasting:
            if topic[0] != "/":
                topic = "/" + topic
            self.create_subscriber(topic, msg_type, trusted)
            if topic == "/receiving":
                rospy.logerror("{}: topic name 'receiving' is reserved".format(
                    self.name))
                continue
            self.conn[topic] = Connection(host, port, "{}{}".format(
                self.name, topic), private_key)
            self.conn[topic].start()
        self.receiver = Connection(host, port, "{}{}".format(
            self.name, "/receiving"), private_key)
        self.descriptionConn = Connection(host, port, "{}/description".format(
            self.name), private_key)
        self.receiver.start()
        self.descriptionConn.start()
        self.timer.start()
        while not rospy.is_shutdown():
            #for key, conn in self.conn.iteritems():
            #    updates = conn.updates()
            updates = self.receiver.updates()
            for v in updates.values():
                self.pub_man.publish(v)
        for key, conn in self.conn.iteritems():
            conn.stop()
	self.receiver.stop()
        self.timer.cancel()
        self.descriptionConn.stop()

    # Creates a subscriber for messages of msg_type published on topic.
    def create_subscriber(self, topic, msg_type, trusted):
        namespace, msg_name = msg_type.split("/")
        mod = __import__(namespace + ".msg")
        msg_cls = getattr(mod.msg, msg_name)
        cb = self.create_callback(topic, msg_type, trusted)
        self.subs[topic] = rospy.Subscriber(topic, msg_cls, cb, None, 1)
        return self

    # Creates a callback function for the subscribers.
    # Formats the packet as a dictionary and sends it to the Connection.
    def create_callback(self, topic, msg_type, trusted):
        def callback(msg):
            data = dict()
            data["To"] = trusted.split(' ')
            data["From"] = self.name
            data["Topic"] = "/{}{}".format(self.name, topic)
            data["Type"] = msg_type
            data["Stamp"] = time.time()
            data["Private_key"] = self.private_key
            data["Msg"] = mc.convert_ros_message_to_dictionary(msg)
            self.conn[topic].send_message(data)
        return callback

    # Periodic function to send the client description.
    def descriptionSend(self):
        data = dict()
        data["To"] = [".*"]
        data["From"] = self.name
        data["Topic"] = "/{}/description".format(self.name)
        data["Type"] = "std_msgs/String"
        data["Stamp"] = time.time()
        data["Private_key"] = self.private_key
        msg = dict()
        msg["data"] = self.description
        data["Msg"] = msg
        self.descriptionConn.send_message(data)
        self.timer = threading.Timer(0.1, self.descriptionSend)
        self.timer.start()

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    name = rospy.get_param("~name")
    topics = rospy.get_param("~publishing", [])
    types = rospy.get_param("~types", [])
    trusted = rospy.get_param("~trusted", [])
    host = rospy.get_param("~host")
    port = rospy.get_param("~port")
    private_key = rospy.get_param("~private_key")
    description = rospy.get_param("~description")
    broadcasting = zip(topics, types, trusted)
    rcn = ROSCloudNode(host, port, name, broadcasting, private_key,
        description)
    rcn.run()
