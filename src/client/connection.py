
import zlib
import threading
import json
import copy
import struct
import rospy
import time
import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.ioloop
from std_msgs.msg import Float32

class Connection(threading.Thread):
    def __init__(self, host, port, name):
        super(Connection, self).__init__()
        self.host = host
        self.port = port
        self.name = name
        self.url = "ws://{}:{}/{}".format(host, port, name)
        self.ioloop = tornado.ioloop.IOLoop.current()
        self.connection = None
        self.values = dict()
        self.acknowledged = True
        self.timer = threading.Timer
        self.freqPub = rospy.Publisher("/{}/ping".format(name), Float32, queue_size=0)

    def run(self):
        tornado.websocket.websocket_connect(
            self.url,
            self.ioloop,
            callback = self.on_connected,
            on_message_callback = self.on_message)
        self.ioloop.start()

    def stop(self):
        self.ioloop.stop()

    def send_message_cb(self, data):
        payload = json.dumps(data)
        frmt = "%ds" % len(payload)
        binary = struct.pack(frmt, payload)
        binLen = len(binary)
        binary = struct.pack('=I' + frmt, binLen, payload)
        compressed = zlib.compress(binary)
        if not self.connection is None:
            # rospy.loginfo(self.acknowledged)
            if self.acknowledged:
                self.acknowledged = False
                if binLen > 1000:
                    self.freqPub.publish(0.0)
                self.connection.write_message(compressed, True)
                self.timer = threading.Timer(1, self.timeout)
                self.timer.start()
    
    def send_message(self, data):
        self.ioloop.add_callback(self.send_message_cb, data)

    def updates(self):
        payloads = copy.copy(self.values)
        self.values = dict()
        return payloads

    def on_connected(self, res):
        try:
            self.connection = res.result()
        except Exception, e:
            print "Failed to connect: {}".format(e)
            tornado.websocket.websocket_connect(
            self.url,
            self.ioloop,
            callback = self.on_connected,
            on_message_callback = self.on_message)


    def on_message(self, payload):
        if len(payload) == 1:
            self.acknowledged = True
            self.timer.cancel()
        else:
            decompressed = zlib.decompress(payload)
            size = struct.unpack('=I', decompressed[:4])
            frmt = "%ds" % size[0]
            unpacked = struct.unpack('=I' + frmt, decompressed)
            data = json.loads(unpacked[1])
            self.values[data["topic"]] = data

    def timeout(self):
        self.acknowledged = True

