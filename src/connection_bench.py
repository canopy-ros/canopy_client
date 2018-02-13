#!/usr/bin/env python

import websocket
import socket
import zlib
import json
import time
import rospy
import struct
from geometry_msgs.msg import PoseStamped
from rospy_message_converter import message_converter as mc

try:
    import thread
except ImportError:
    import _thread as thread
import time

NODE_NAME = "ws_benchmark"
ROBOT_NAME = "foo"
HZ = 20


def on_message(ws, message):
    # print(message)
    pass


def on_error(ws, error):
    print(error)


def on_close(ws):
    print("### closed ###")


def on_open(ws):
    def run(*args):
        r = rospy.Rate(HZ)
        seq = 0
        count = 0
        start = time.time()
        while not rospy.is_shutdown():
            msg = gen_message(seq)
            data = compress_message(msg)
            ws.send(data)
            r.sleep()
            count += 1
            if count >= 30:
                end = time.time()
                rospy.loginfo(30.0 / (end - start))
                start = end
                count = 0
            seq += 1
        time.sleep(1)
        ws.close()
        print("thread terminating...")
    thread.start_new_thread(run, ())


def gen_message(seq):
    msg = PoseStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "map"
    msg.header.seq = seq
    msg.pose.position.x = 1
    msg.pose.position.y = 2
    msg.pose.position.z = 3
    msg.pose.orientation.w = 1
    msg_data = mc.convert_ros_message_to_dictionary(msg)
    data = {
        "To": ["bar"],
        "From": ROBOT_NAME,
        "Topic": "/foo/state",
        "Type": "geometry_msgs/PoseStamped",
        "Stamp": time.time(),
        "PrivateKey": "abc",
        "Msg": msg_data,
    }
    return data


def compress_message(data):
    payload = json.dumps(data)
    frmt = "%ds" % len(payload)
    binary = struct.pack(frmt, payload)
    binLen = len(binary)
    binary = struct.pack('=I' + frmt, binLen, payload)
    compressed = zlib.compress(binary)
    return compressed


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    #websocket.enableTrace(True)
    host = "wallar.csail.mit.edu"
    port = 8080
    name = "foo/state"
    private_key = "abc"
    url = "ws://{}:{}/{}/{}".format(host, port, private_key, name)
    rospy.loginfo(url)
    ws = websocket.WebSocketApp(
        url,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )
    ws.on_open = on_open
    ws.run_forever(
        sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),
                 (socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1))
    )
