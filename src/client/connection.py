
import zlib
import threading
import json
import copy
import struct
from twisted.internet import reactor
from twisted.internet.protocol import ReconnectingClientFactory
from autobahn.twisted.websocket import WebSocketClientProtocol
from autobahn.twisted.websocket import WebSocketClientFactory
import rospy


class MMClient(WebSocketClientProtocol):

    client = None
    updates = dict()
    acknowledged = True
    timer = threading.Timer

    def onConnect(self, reponse):
        MMClient.client = self
        MMClient.acknowledged = True
        MMClient.timer = threading.Timer

    def onMessage(self, payload, is_binary):
        if not is_binary:
            data = json.loads(payload)
            MMClient.updates[data["topic"]] = data
        else:
            if len(payload) == 1:
                MMClient.acknowledged = True
                MMClient.timer.cancel()
            else:
                decompressed = zlib.decompress(payload)
                size = struct.unpack('=I', decompressed[:4])
                frmt = "%ds" % size[0]
                unpacked = struct.unpack('=I' + frmt, decompressed)
                data = json.loads(unpacked[1])
                MMClient.updates[data["topic"]] = data

    def onClose(self, wasClean, code, reason):
        rospy.logwarn("WebSocket connection closed: {0}".format(reason))

    @staticmethod
    def timeout():
        MMClient.acknowledged = True

    @staticmethod
    def send_message(payload, is_binary):
        if not MMClient.client is None:
            # rospy.loginfo(MMClient.acknowledged)
            if MMClient.acknowledged:
                MMClient.acknowledged = False
                MMClient.client.sendMessage(payload, is_binary)
                MMClient.timer = threading.Timer(1, MMClient.timeout)
                MMClient.timer.start()


class ClientFactory(WebSocketClientFactory, ReconnectingClientFactory):
    def clientConnectionFailed(self, connector, reason):
        print "Connection Failed {} -- {}".format(connector, reason)

    def clientConnectionLost(self, connector, reason):
        print "Connection Failed {} -- {}".format(connector, reason)


class Connection(threading.Thread):
    def __init__(self, host, port, name):
        super(Connection, self).__init__()
        self.host = host
        self.port = port
        self.name = name
        self.url = "ws://{}:{}/{}".format(host, port, name)
        self.factory = ClientFactory(self.url, debug=False)
        self.daemon = True

    def run(self):
        self.factory.protocol = MMClient
        reactor.connectTCP(self.host, self.port, self.factory)
        reactor.run(installSignalHandlers=0)

    def stop(self):
        reactor.stop()

    def send_message(self, data):
        payload = json.dumps(data)
        frmt = "%ds" % len(payload)
        binary = struct.pack(frmt, payload)
        binLen = len(binary)
        binary = struct.pack('=I' + frmt, binLen, payload)
        compressed = zlib.compress(binary)
        return MMClient.send_message(compressed, True)

    def updates(self):
        payloads = copy.copy(MMClient.updates)
        MMClient.updates = dict()
        return payloads
