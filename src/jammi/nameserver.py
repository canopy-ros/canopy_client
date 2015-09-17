
import socket
import zlib
import time
import json
import threading


class NameServer(object):
    """
    A name server class that is used to aggregate masters that are
    sending heartbeat messages over the network. This class is used
    for the discovery of other masters
    """

    def __init__(self, host, port, interval):
        """
        Initializes a NameServer

        Parameters
        ----------
        host: string
            The host of the multicast group

        port: integer
            The port of the multicast group

        interval: number
            The amount of time that can elapse between heartbeat messages
            before the remote master is labeled "dead"
        """

        self.host = host
        self.port = port
        self.interval = interval
        self.masters = dict()
        self.times = dict()
        self.new_connections = list()
        self.running = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))

    def kill(self):
        """
        Kills the name server thread
        """

        self.running = False
        return self

    def get_alive(self):
        """
        Returns the names of masters that are still labeled as "alive"

        Returns
        -------
        alive: list of strings
            A list of master names that are still alive
        """

        alive = list()
        ctime = time.time()
        for name, t in self.times.iteritems():
            if ctime - t < self.interval:
                alive.append(name)
        return alive

    def get_data(self, name):
        """
        Gets the data associated with a given master name

        Parameters
        ----------
        name: string
            The name of the master

        Returns
        -------
        A: dict
            The associated data of the master
        """

        return self.masters[name]

    def get_time(self, name):
        """
        Gets the last time that the master with name, `name`, sent a hearbeat
        message

        Parameters
        ----------
        name: string
            The name of the master

        Returns
        -------
        A: float
            The last time the master as sent a heartbeat message
        """

        return self.times[name]

    def discover(self):
        """
        Discover new masters

        Returns
        -------
        nc: list of strings
            The new masters on the network

        Notes
        -----
        This method resets the `new_connections` variable to an empty list
        """

        nc = self.new_connections[:]
        self.new_connections = list()
        return nc

    def start(self):
        """
        Starts the name server thread

        Notes
        -----
        If the name server is already running, a RuntimeWarning will be
        thrown
        """

        if self.running:
            raise RuntimeWarning("NameServer already running")
        else:
            def __thread():
                while self.running:
                    data_zip, _ = self.sock.recvfrom(1024)
                    data_str = zlib.decompress(data_zip)
                    data = json.loads(data_str)
                    if not data["name"] in self.masters:
                        self.new_connections.append(data["name"])
                    self.masters[data["name"]] = data
                    self.times[data["name"]] = time.time()
            self.running = True
            self.thread = threading.Thread(target=__thread)
            self.thread.daemon = True
            self.thread.start()
