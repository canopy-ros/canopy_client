
import zlib
import json
import rospy
import time
import common
import struct
import multiprocessing
from std_msgs.msg import Float32
import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.ioloop
from tornado import gen
from concurrent.futures import ProcessPoolExecutor

def call_process(message):
    MMServerProtocol.instance.process_message(message)

class MMServerProtocol(tornado.websocket.WebSocketHandler):
    lat_pubs = dict()

    def open(self, name):
        common.add_client(name, self)
        self.name_of_client = name
        MMServerProtocol.lat_pubs[name] = rospy.Publisher("/jammi/" + name
                                      + "/latency", Float32, queue_size=2)
        print "Connected to: {}".format(name)
        self.ioloop = tornado.ioloop.IOLoop.instance()
        MMServerProtocol.instance = self
        self.pool = multiprocessing.Pool(multiprocessing.cpu_count())

    def send_to_client(self, to, message):
        to.write_message(message, True)

    def process_message(self, message, callback):
        received_time = time.time()
        decompressed = zlib.decompress(message)
        size = struct.unpack('=I', decompressed[:4])
        frmt = "%ds" % size[0]
        unpacked = struct.unpack('=I' + frmt, decompressed)
        msg = json.loads(unpacked[1])
        acknowledge = struct.pack('=b', 0)
        self.ioloop.add_callback(
            self.send_to_client,
            common.get_client(msg["from"]), acknowledge)
        latency = Float32()
        latency.data = received_time - msg["stamp"]
        MMServerProtocol.lat_pubs[msg["from"]].publish(latency)
        if msg["to"][0] == "*":
            for name in common.clients.keys():
                if name != msg["from"]:
                    self.ioloop.add_callback(
                        self.send_to_client,
                        common.get_client(name), message)
        else:
            for name in msg["to"]:
                self.ioloop.add_callback(
                        self.send_to_client,
                        common.get_client(name), message)
        return callback(message)

    @tornado.gen.coroutine
    def on_message(self, message):
        print len(message)
        result = yield tornado.gen.Task(self.process_message, message)
        return
        pool = ProcessPoolExecutor()
        fut = pool.submit(call_process, message)
        ret = yield fut
        pool.shutdown()

    def on_close(self):
        common.remove_client(self.name_of_client)
