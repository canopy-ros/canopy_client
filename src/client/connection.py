
# import zlib
import threading
import json
import copy
from twisted.internet import reactor
from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory


class MMClient(WebSocketClientProtocol):

    client = None
    updates = dict()

    def onConnect(self, reponse):
        MMClient.client = self

    def onMessage(self, payload, is_binary):
        if not is_binary:
            data = json.loads(payload)
            MMClient.updates[data["topic"]] = data

    @staticmethod
    def send_message(payload, is_binary):
        if not MMClient.client is None:
            MMClient.client.sendMessage(payload, is_binary)


class Connection(threading.Thread):
    def __init__(self, host, port, name):
        super(Connection, self).__init__()
        self.host = host
        self.port = port
        self.name = name
        self.url = "ws://{}:{}/{}".format(host, port, name)
        self.factory = WebSocketClientFactory(self.url, debug=True)
        self.daemon = True

    def run(self):
        self.factory.protocol = MMClient
        reactor.connectTCP(self.host, self.port, self.factory)
        reactor.run(installSignalHandlers=0)

    def stop(self):
        reactor.stop()

    def send_message(self, data):
        payload = json.dumps(data)
        is_binary = False
        return MMClient.send_message(payload, is_binary)

    def updates(self):
        payloads = copy.copy(MMClient.updates)
        MMClient.updates = dict()
        return payloads
