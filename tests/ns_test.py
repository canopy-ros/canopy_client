
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "../src"))

import jammi
import rospy


def test_start():
    """
    Tests that the NameServer is able to store "alive" masters
    """

    rospy.init_node("jammi_test", anonymous=False)
    host, port = "localhost", 5005
    rate = 20
    ns = jammi.NameServer(host, port, 3)
    ns.start()
    heart = jammi.Heart(host, port, rate)
    heart.data["name"] = "test"
    heart.start()
    r = rospy.Rate(10)
    alive_set = set()
    for i in xrange(5):
        alive_set.update(set(ns.get_alive()))
        r.sleep()
    assert(list(alive_set)[0] == heart.data["name"])
    heart.kill()
    ns.kill()


if __name__ == "__main__":
    test_start()
