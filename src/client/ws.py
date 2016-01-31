
import threading
import string
import sys
from twisted.internet import reactor
from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory


test_msg = """
{"to": "$to",
 "from": "$fr",
 "topic": "/foo/state",
 "type": "geometry_msgs/Point",
 "msg": "{'x': 0, 'y': 0, 'z': 0}",
 "stamp": $t}
"""


class MMClient(WebSocketClientProtocol):

    client = None

    def onConnect(self, reponse):
        MMClient.client = self

    def onMessage(self, payload, is_binary):
        if not is_binary:
            print payload

    @staticmethod
    def send_message(payload):
        if not MMClient.client is None:
            MMClient.client.sendMessage(payload)


class Connection(threading.Thread):
    def __init__(self, host, port, name):
        super(Connection, self).__init__()
        self.host = host
        self.port = port
        self.name = name
        self.url = "ws://{}:{}/{}".format(host, port, name)
        self.factory = WebSocketClientFactory(self.url, debug=True)

    def run(self):
        self.factory.protocol = MMClient
        reactor.connectTCP(self.host, self.port, self.factory)
        reactor.run(installSignalHandlers=0)

    def stop(self):
        reactor.stop()

    def send_message(self, payload):
        return MMClient.send_message(payload)


if __name__ == "__main__":
    import time
    conn = Connection("localhost", 9000, sys.argv[1])
    conn.daemon = True
    conn.start()
    msg_tmp = string.Template(test_msg)
    while True:
        msg = msg_tmp.substitute(
            fr=sys.argv[1], to=sys.argv[2], t=time.time())
        conn.send_message(msg)
        time.sleep(0.1)
