import rospy
import unittest

from controller import Controller

class TestController(unittest.TestCase):
    def setup(self):
        rospy.init_node('test_node', anonymous=True)
        self.controller = Controller()
    
    def test_init(self):
        assert self.controller._config is not None

if __name__ == "__main__":
    unittest.main()
