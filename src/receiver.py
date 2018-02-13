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
class Receiver(threading.Thread):

    def __init__(self, socket):
        super(Receiver, self).__init__()
        self.socket = socket
        self.values = dict()

    # Starts the Tornado IOLoop and connects to the websocket.
    # Called on thread start.
    def run(self):
        while True:
            data = self.socket.recv(65565)
            if data == "HANDSHAKE":
                continue
            self.process_message(data)

    # Returns the formatted last received message.
    def updates(self):
        #payloads = copy.copy(self.values)
        payloads = self.values
        self.values = dict()
        return payloads

    # Callback for message receiving.
    # Decompresses messages, converts to unicode,
    # and converts from JSON to dictionary.
    def process_message(self, payload):
        try:
            decompressed = zlib.decompress(payload)
            size = struct.unpack('=I', decompressed[:4])
            frmt = "%ds" % size[0]
            unpacked = struct.unpack('=I' + frmt, decompressed)
            data = json.loads(unpacked[1])
            self.values[data["Topic"]] = data
        except:
            pass
