
import zlib
import json
import rospy
import time
import common
import struct
from std_msgs.msg import Float32
import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.ioloop


class MMServerProtocol(tornado.websocket.WebSocketHandler):
    lat_pubs = dict()

    def open(self, name):
        common.add_client(name, self)
        self.name_of_client = name
        MMServerProtocol.lat_pubs[name] = rospy.Publisher("/jammi/" + name
                                      + "/latency", Float32, queue_size=2)
        print "Connected to: {}".format(name)


    def on_message(self, message):
        received_time = time.time()
        decompressed = zlib.decompress(message)
        size = struct.unpack('=I', decompressed[:4])
        frmt = "%ds" % size[0]
        unpacked = struct.unpack('=I' + frmt, decompressed)
        msg = json.loads(unpacked[1])
        acknowledge = struct.pack('=b', 0)
        self.write_message(acknowledge, True)
        latency = Float32()
        latency.data = received_time - msg["stamp"]
        MMServerProtocol.lat_pubs[msg["from"]].publish(latency)
        if msg["to"][0] == "*":
            for name in common.clients.keys():
                if name != msg["from"]:
                    common.get_client(name).write_message(message, True)
        else:
            for name in msg["to"]:
                common.get_client(name).write_message(message, True)

    def on_close(self):
        common.remove_client(self.name_of_client)
