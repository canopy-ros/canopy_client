# Defines the Connection class.

import zlib
import threading
import json
import copy
import struct
import rospy
import time
import socket

# Represents a threaded websocket connection to the server.
class Sender():
    lock = threading.Lock()

    def __init__(self, socket):
        self.socket = socket
        self.data = None
        self.worker = None
        self.values = dict()

    # Starts the Tornado IOLoop and connects to the websocket.
    # Called on thread start.
    def run(self):
        self.send_message_cb(self.data)

    # Formats data dictionary as JSON, converts to binary,
    # compresses using zlib, and sends to the server.
    def send_message_cb(self, data):
        payload = json.dumps(data)
        frmt = "%ds" % len(payload)
        binary = struct.pack(frmt, payload)
        binLen = len(binary)
        binary = struct.pack('=I' + frmt, binLen, payload)
        compressed = zlib.compress(binary)
        with Sender.lock:
            self.socket.sendall(compressed)

    # Creates callback to send message in IOLoop.
    def send_message(self, data):
        self.data = data
        self.worker = threading.Thread(target=self.run)
        self.worker.start()
