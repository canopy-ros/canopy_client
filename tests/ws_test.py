
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "../src"))

import server
import time
from twisted.internet import task
from twisted.python import log
from twisted.internet import reactor
from autobahn.twisted.websocket import WebSocketServerFactory
from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory


test_msg = """
{"to": "bar",
 "from": "foo",
 "topic": "/foo/state",
 "type": "geometry_msgs/Point",
 "msg": "{'x': 0, 'y': 0, 'z': 0}",
 "stamp": 0}
"""


class ClientProtocol(WebSocketClientProtocol):

    def onConnect(self, response):
        print("Server connected: {0}".format(response.peer))

    def onOpen(self):
        print("WebSocket connection open.")

        def hello():
            self.sendMessage(test_msg)
            self.factory.reactor.callLater(1, hello)

        hello()

    def onMessage(self, payload, is_binary):
        if not is_binary:
            print payload

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))


def print_conns():
    print server.clients


def create_websocket():
    factory = WebSocketClientFactory(u"ws://127.0.0.1:9000/foo", debug=True)
    factory.protocol = ClientProtocol
    reactor.connectTCP("127.0.0.1", 9000, factory)

    factory = WebSocketClientFactory(u"ws://127.0.0.1:9000/bar", debug=True)
    factory.protocol = ClientProtocol
    reactor.connectTCP("127.0.0.1", 9000, factory)



def test_ws_server():
    log.startLogging(sys.stdout)
    # create_websocket()
    factory = WebSocketServerFactory("ws://localhost:9000", debug=True)
    factory.protocol = server.MMServerProtocol
    reactor.listenTCP(9000, factory)
    # task.LoopingCall(print_conns).start(1.0)
    reactor.run()
