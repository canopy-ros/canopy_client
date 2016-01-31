
import json
import common
from autobahn.twisted.websocket import WebSocketServerProtocol


class MMServerProtocol(WebSocketServerProtocol):

    def onConnect(self, request):
        name = request.path[1:]
        common.add_client(name, self)
        self.name_of_client = name

    def onOpen(self):
        pass

    def onMessage(self, payload, is_binary):
        if not is_binary:
            msg = json.loads(payload)
            if msg["to"] == "*":
                for name in common.clients.keys():
                    common.get_client(msg["to"]).sendMessage(payload)
            else:
                common.get_client(msg["to"]).sendMessage(payload)

    def onClose(self, was_clean, code, reason):
        common.remove_client(self.name_of_client)
