# Defines the Connection class.

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

# Represents a threaded websocket connection to the server.
class Connection(threading.Thread):

    def __init__(self, host, port, name, private_key):
        super(Connection, self).__init__()
        self.host = host
        self.port = port
        self.name = name
        self.url = "ws://{}:{}/{}/{}".format(host, port, private_key, name)
        self.ioloop = None
        self.connection = None
        self.values = dict()
        self.acknowledged = True
        self.timer = threading.Timer

    # Starts the Tornado IOLoop and connects to the websocket.
    # Called on thread start.
    def run(self):
        while self.ioloop is None:
            self.ioloop = tornado.ioloop.IOLoop()
        tornado.websocket.websocket_connect(
            self.url,
            self.ioloop,
            callback = self.on_connected,
            on_message_callback = self.on_message)
        self.ioloop.start()

    # Stops the IOLoop
    def shutdown(self):
        self.ioloop.stop()

    # Called on thread abortion
    def stop(self):
        self.ioloop.add_callback(self.shutdown)

    # Formats data dictionary as JSON, converts to binary,
    # compresses using zlib, and sends to the server.
    def send_message_cb(self, data):
        payload = json.dumps(data)
        frmt = "%ds" % len(payload)
        binary = struct.pack(frmt, payload)
        binLen = len(binary)
        binary = struct.pack('=I' + frmt, binLen, payload)
        compressed = zlib.compress(binary)
        if not self.connection is None:
            if self.acknowledged:
                self.acknowledged = False
                self.connection.write_message(compressed, True)
                self.timer = threading.Timer(1, self.timeout)
                self.timer.start()
    
    # Creates callback to send message in IOLoop.
    def send_message(self, data):
	if not self.ioloop is None:
        	self.ioloop.add_callback(self.send_message_cb, data)

    # Returns the formatted last received message.
    def updates(self):
        payloads = copy.copy(self.values)
        self.values = dict()
        return payloads

    # Callback for websocket connection. Retries if connection fails.
    def on_connected(self, res):
        try:
            self.connection = res.result()
            while self.connection is None:
                tornado.websocket.websocket_connect(
                    self.url,
                    self.ioloop,
                    callback = self.on_connected,
                    on_message_callback = self.on_message)
        except Exception, e:
            print "Failed to connect: {}".format(e)
            tornado.websocket.websocket_connect(
            self.url,
            self.ioloop,
            callback = self.on_connected,
            on_message_callback = self.on_message)

    # Callback for message receiving.
    # Detects between websocket close, acknowledge packet, or message packet.
    # Decompresses messages, converts to unicode,
    # and converts from JSON to dictionary.
    def on_message(self, payload):
        if payload is None:
            self.connection = None
            print "Server connection closed. Reconnecting..."
            tornado.websocket.websocket_connect(
                    self.url,
                    self.ioloop,
                    callback = self.on_connected,
                    on_message_callback = self.on_message)
        if len(payload) == 1:
            self.acknowledged = True
            try:
                self.timer.cancel()
            except:
                pass
        else:
            decompressed = zlib.decompress(payload)
            size = struct.unpack('=I', decompressed[:4])
            frmt = "%ds" % size[0]
            unpacked = struct.unpack('=I' + frmt, decompressed)
            data = json.loads(unpacked[1])
            self.values[data["Topic"]] = data

    # Timeout for acknowledge packet.
    def timeout(self):
        self.acknowledged = True

