#!/usr/bin/env python
# Defines the CanopyClientNode class.

import rospy
import time
import requests
import publishermanager as pm
from rospy_message_converter import message_converter as mc
from connection import Connection
import threading


NODE_NAME = "canopy_client"


class CanopyClientNode(object):
    """
    The ROS node object for the Canopy client.
    Manages all connections and subscribing.
    One instance per client node.
    """
    def __init__(self, host, port, name, broadcasting, private_key,
                 description, global_frames, leaflets, use_local_time):
        self.host = host
        self.port = port
        self.name = name.replace(" ", "").replace("/", "")
        self.use_local_time = use_local_time
        self.conn = dict()
        self.receiver = None
        self.descriptionConn = None
        self.subs = dict()
        self.broadcasting = broadcasting
        self.private_key = private_key
        self.description = description
        self.global_frames = global_frames
        self.leaflets = leaflets
        self.pub_man = pm.PublisherManager(use_local_time)
        self.timer = threading.Timer(0.1, self.descriptionSend)

    def post_leaflet_urls(self):
        if len(self.leaflets) > 0:
            payload = {"urls": self.leaflets}
            post_url = "http://{}:{}/{}/leaflets".format(
                self.host, self.port, self.private_key)
            r = requests.post(post_url, data=payload)
            return r

    # Creates all connections and subscribers and starts them.
    # Runs a loop that checks for received messages.
    def run(self):
        for topic, msg_type, trusted in self.broadcasting:
            if topic[0] != "/":
                topic = "/" + topic
            self.create_subscriber(topic, msg_type, trusted)
            if topic == "/receiving":
                rospy.logerr("{}: topic name 'receiving' is reserved".format(
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
        #self.post_leaflet_urls()
        while not rospy.is_shutdown():
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
        self.subs[topic] = rospy.Subscriber(topic, msg_cls, cb, queue_size=1)
        return self

    # Creates a callback function for the subscribers.
    # Formats the packet as a dictionary and sends it to the Connection.
    def create_callback(self, topic, msg_type, trusted):
        def callback(msg):
            if msg._connection_header["callerid"] == rospy.get_name():
                return
            data = dict()
            data["To"] = trusted.split(' ')
            data["From"] = self.name
            if topic == "/tf":
                data["Topic"] = topic
            else:
                data["Topic"] = "/{}{}".format(self.name, topic)
            data["Type"] = msg_type
            data["Stamp"] = time.time()
            data["PrivateKey"] = self.private_key
            if msg_type == "tf2_msgs/TFMessage":
                for t in msg.transforms:
                    t = self.modify_stamped_message(t)
            else:
                msg = self.modify_stamped_message(msg)
            data["Msg"] = mc.convert_ros_message_to_dictionary(msg)
            self.conn[topic].send_message(data)
        return callback

    def modify_child_frame_id(self, message):
        if (message.child_frame_id.find("/") > 0 or
                message.child_frame_id.count("/") > 1):
            return message
        if message.child_frame_id not in self.global_frames:
            if message.child_frame_id[0] != "/":
                message.child_frame_id = "/" + message.child_frame_id
            message.child_frame_id = "{}{}".format(self.name,
                                                   message.child_frame_id)

    def modify_header_frame_id(self, message):
        if ((not hasattr(message, 'child_frame_id')) and
                message.header.frame_id.find("/") > 0 and
                message.header.frame_id.count("/") > 1):
            return message
        if message.header.frame_id not in self.global_frames:
            if (len(message.header.frame_id) > 0 and
                message.header.frame_id.find("/") <= 0 and
                    message.header.frame_id.count("/") <= 1):
                if message.header.frame_id[0] != "/":
                    message.header.frame_id = "/" + message.header.frame_id
                message.header.frame_id = "{}{}".format(
                    self.name, message.header.frame_id)

    def modify_stamped_message(self, message):
        if hasattr(message, 'child_frame_id'):
            self.modify_child_frame_id(message)

        if hasattr(message, 'header'):
            self.modify_header_frame_id(message)
        return message

    # Periodic function to send the client description.
    def descriptionSend(self):
        data = dict()
        data["To"] = [".*"]
        data["From"] = self.name
        data["Topic"] = "/{}/description".format(self.name)
        data["Type"] = "std_msgs/String"
        data["Stamp"] = time.time()
        data["PrivateKey"] = self.private_key
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
    global_frames = rospy.get_param("~global_frames", [])
    leaflets = rospy.get_param("~leaflets", [])
    use_local_time = rospy.get_param("~use_local_time", False)
    host = rospy.get_param("~host")
    port = rospy.get_param("~port")
    private_key = rospy.get_param("~private_key")
    description = rospy.get_param("~description")
    broadcasting = zip(topics, types, trusted)
    rcn = CanopyClientNode(host, port, name, broadcasting, private_key,
                           description, global_frames, leaflets,
                           use_local_time)
    rcn.run()
