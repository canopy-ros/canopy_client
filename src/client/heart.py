
import socket
import json
import zlib
import rospy
import threading


class Heart(object):
    """
    A heartbeat class that can be used to send messages over UDP to a group
    with a given rate
    """

    def __init__(self, host, port, rate):
        """
        Initializes a multicast heart object

        Parameters
        ----------
        host: string
            The host of the multicast group

        port: integer
            The port of the multicast group

        rate: integer
            The rate per second that the messages would be sent over UDP
        """

        self.host = host
        self.port = port
        self.rate = rate
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data = dict()
        self.running = False

    def set_data(self, data):
        """
        Sets the data being sent over UDP

        Parameters
        ----------
        data: dict
            A dictionary to update the current data with
        """

        self.data = data
        return self

    def beat(self):
        """
        Compresses and sends the formatted data over UDP

        Notes
        -----
        This is not to be called independently but instead gets called by
        the timed thread in `start`
        """
        json_str = json.dumps(self.data)
        zip_str = zlib.compress(json_str)
        self.sock.sendto(zip_str, (self.host, self.port))
        return self

    def kill(self):
        """
        Kills the heartbeat thread
        """
        self.running = False
        return self

    def start(self):
        """
        Starts the hearbeat thread

        Notes
        -----
        If the thread has already started, this will throw a RuntimeWarning
        """
        if self.running:
            raise RuntimeWarning("Heart already running")
        else:
            def __thread():
                rate = rospy.Rate(self.rate)
                while self.running:
                    self.beat()
                    rate.sleep()
            self.running = True
            self.thread = threading.Thread(target=__thread)
            self.thread.daemon = True
            self.thread.start()
