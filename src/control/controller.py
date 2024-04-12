import rospy

class Controller:

    def __init__(self):
        self._config = self._load_config()

    def _load_config(self):
        print("Not implemented. Get parameters from rospy.")
        return 1

    def start(self):
        rospy.spin()
    
    def main():
        rospy.init_node('controller')
        c = Controller()
        c.start()
