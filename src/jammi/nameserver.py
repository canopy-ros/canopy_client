
import socket
import zlib
import time
import json
import threading


class NameServer(object):

    def __init__(self, host, port, interval):
        self.host = host
        self.port = port
        self.interval = interval
        self.masters = dict()
        self.times = dict()
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))

    def kill():
        self.running = False
        return self

    def start():
        if self.running:
            raise RuntimeWarning("NameServer already running")
        else:
            def __thread():
                while self.running:
                    data_zip, addr = sock.recvfrom(1024)
                    data_str = zlib.decompress(data_zip)
                    data = json.loads(data_str)
                    self.master[data["name"]] = data
                    self.times[data["name"]] = time.time()
            self.running = True
            self.thread = threading.Thread(target=__thread)
            self.thread.daemon = True
            self.thread.start()
