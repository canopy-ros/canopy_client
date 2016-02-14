#!/usr/bin/env python

import connection
import rospy
import signal
import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.ioloop


NODE_NAME = "jammi_server"

settings = {'debug': False}
app = tornado.web.Application([
    (r'/(.*)', connection.MMServerProtocol),
    ], **settings)

def sig_handler(sig, frame):
    tornado.ioloop.IOLoop.current().add_callback(shutdown)

def shutdown():
    tornado.ioloop.IOLoop.current().stop()

def run_server(host, port):
    url = "ws://{}:{}".format(host, port)
    http_server = tornado.httpserver.HTTPServer(app)
    http_server.bind(port)
    http_server.start(0)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    host = rospy.get_param("~host", "localhost")
    port = rospy.get_param("~port", 9000)
    signal.signal(signal.SIGTERM, sig_handler)
    signal.signal(signal.SIGINT, sig_handler)
    run_server(host, port)
