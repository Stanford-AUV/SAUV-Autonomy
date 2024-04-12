import rospy

class Controller:

    def __init__(self):
        return

    def start(self):
        rospy.spin()

    def main():
        rospy.init_node('controller')
        c = Controller()
        c.start()
