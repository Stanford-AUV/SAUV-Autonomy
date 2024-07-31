import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import Image

class MapperObjectSubscriber(Node):
    def __init__(self):
        super().__init__("mapper_object_subscriber")

        self.buoy_subscription_ = self.create_subscription(
            Object, "", self.buoy_callback, 10
        )

        self.blue_arrow_subscription_ = self.create_subscription(
            Object, "", self.red_arrow_callback, 10
        )

        self.red_arrow_subscription_ = self.create_subscription(
            Object, "", self.blue_arrow_callback, 10
        )

    def buoy_callback(self, msg):
        pass

    def red_arrow_callback(self, msg):
        pass

    def blue_arrow_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    mapper_object_subscriber = MapperObjectSubscriber()

    rclpy.spin(mapper_object_subscriber)

    # Destroy the node explicitly
    mapper_object_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
