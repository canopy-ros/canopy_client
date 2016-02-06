#!/usr/bin/env python

import sys
import ws
import rospy
from twisted.python import log
from twisted.internet import reactor
from autobahn.twisted.websocket import WebSocketServerFactory


NODE_NAME = "jammi_server"


def run_server(host, port):
    #log.startLogging(sys.stdout)
    url = "ws://{}:{}".format(host, port)
    factory = WebSocketServerFactory(url, debug=True)
    factory.protocol = ws.MMServerProtocol
    reactor.listenTCP(port, factory)
    while not rospy.is_shutdown():
        reactor.iterate()


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 9000)
    run_server(host, port)
