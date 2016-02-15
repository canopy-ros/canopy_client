#!/usr/bin/env python

import rospy
import time
import publishermanager as pm
from rospy_message_converter import message_converter as mc
from connection import Connection


NODE_NAME = "roscloud_client"


class ROSCloudNode(object):

    def __init__(self, host, port, name, broadcasting):
        self.host = host
        self.port = port
        self.name = name
        self.conn = dict()
        self.receiver = None
        self.subs = dict()
        self.broadcasting = broadcasting
        self.pub_man = pm.PublisherManager()

    def run(self):
        for topic, msg_type, trusted in self.broadcasting:
            self.create_subscriber(topic, msg_type, trusted)
            if topic == "/receiving":
                rospy.logerror("{}: topic name 'receiving' is reserved".format(
                    self.name))
                continue
            self.conn[topic] = Connection(host, port, "{}{}".format(
                self.name, topic))
            self.conn[topic].start()
        self.receiver = Connection(host, port, "{}{}".format(
            self.name, "/receiving"))
        self.receiver.start()
        while not rospy.is_shutdown():
            #for key, conn in self.conn.iteritems():
            #    updates = conn.updates()
            updates = self.receiver.updates()
            for v in updates.values():
                self.pub_man.publish(v)
        for key, conn in self.conn.iteritems():
            conn.stop()
	self.receiver.stop()

    def create_subscriber(self, topic, msg_type, trusted):
        namespace, msg_name = msg_type.split("/")
        mod = __import__(namespace + ".msg")
        msg_cls = getattr(mod.msg, msg_name)
        cb = self.create_callback(topic, msg_type, trusted)
        self.subs[topic] = rospy.Subscriber(topic, msg_cls, cb, None, 1)
        return self

    def create_callback(self, topic, msg_type, trusted):
        def callback(msg):
            data = dict()
            data["To"] = trusted.split(' ')
            data["From"] = self.name
            data["Topic"] = "/{}{}".format(self.name, topic)
            data["Type"] = msg_type
            data["Stamp"] = time.time()
            data["Msg"] = mc.convert_ros_message_to_dictionary(msg)
            self.conn[topic].send_message(data)
        return callback


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    name = rospy.get_param("~name")
    topics = rospy.get_param("~publishing", [])
    types = rospy.get_param("~types", [])
    trusted = rospy.get_param("~trusted", [])
    host = rospy.get_param("~host")
    port = rospy.get_param("~port")
    broadcasting = zip(topics, types, trusted)
    rcn = ROSCloudNode(host, port, name, broadcasting)
    rcn.run()
