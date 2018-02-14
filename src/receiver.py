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
        self.running = False

    # Starts the Tornado IOLoop and connects to the websocket.
    # Called on thread start.
    def run(self):
        self.running = True
        while self.running:
            try:
                data = self.socket.recv(65565)
                if data == "HANDSHAKE":
                    continue
                self.process_message(data)
            except socket.timeout:
                rospy.logwarn("[canopy-client] Data recv timed out")

    def stop(self):
        self.running = False

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
        except Exception as e:
            rospy.logerr(e)
