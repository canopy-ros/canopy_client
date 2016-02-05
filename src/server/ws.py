
# import zlib
import json
import rospy
import time
import common
from std_msgs.msg import Float32
from autobahn.twisted.websocket import WebSocketServerProtocol


class MMServerProtocol(WebSocketServerProtocol):

    def onConnect(self, request):
        name = request.path[1:]
        common.add_client(name, self)
        self.name_of_client = name
        self.lat_pub = rospy.Publisher("/jammi/latency", Float32, queue_size=2)

    def onMessage(self, payload, is_binary):
        if not is_binary:
            try:
                msg = json.loads(payload)
                latency = Float32()
                latency.data = time.time() - msg["stamp"]
                self.lat_pub.publish(latency)
                if msg["to"] == "*":
                    for name in common.clients.keys():
                        if name != msg["from"]:
                            common.get_client(name).sendMessage(payload)
                else:
                    common.get_client(msg["to"]).sendMessage(payload)
            except KeyError:
                pass

    def onClose(self, was_clean, code, reason):
        common.remove_client(self.name_of_client)
