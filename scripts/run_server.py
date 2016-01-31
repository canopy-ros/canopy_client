
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "../src"))

import server
from twisted.python import log
from twisted.internet import reactor
from autobahn.twisted.websocket import WebSocketServerFactory


def run_server():
    log.startLogging(sys.stdout)
    factory = WebSocketServerFactory("ws://localhost:9000", debug=True)
    factory.protocol = server.MMServerProtocol
    reactor.listenTCP(9000, factory)
    reactor.run()


if __name__ == "__main__":
    run_server()
