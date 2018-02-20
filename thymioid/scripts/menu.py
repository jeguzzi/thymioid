import rospy
from std_msgs.msg import Int8


class Menu(object):
    """docstring for Menu."""

    def __init__(self, index, size):
        super(Menu, self).__init__()

        self._config = None
        self.size = size
        self.index = index

        self.pub_value = rospy.Publisher('config/{0}'.format(index), Int8, queue_size=1,
                                         latch=True)
        self.pub_size = rospy.Publisher('config/{0}/size'.format(index), Int8, queue_size=1,
                                        latch=True)
        rospy.Subscriber('config/{0}/target'.format(index), Int8, self.on_target, queue_size=1)
        self.pub_size.publish(size)

    def on_target(self, msg):
        value = msg.data
        if value != self._config and value < self.size:
            self.target_config(value)

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        if value != self._config and value < self.size:
            self._config = value
            self.pub_value.publish(value)
