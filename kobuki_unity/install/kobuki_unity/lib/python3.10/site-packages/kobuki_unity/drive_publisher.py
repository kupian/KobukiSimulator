import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ugrdv_kobuki_msgs.msg import DriveCommand

class DrivePublisher(Node):
    def __init__(self):
        super().__init__("drive_publisher")
        self.test_publisher = self.create_publisher(DriveCommand, "/ugrdv_kobuki/drive_command", 1)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        drive_command = DriveCommand()
        drive_command.linearvel = 1.0
        drive_command.angularvel = 1.5
        self.test_publisher.publish(drive_command)
        self.get_logger().info("Publishing drive command")

def main(args=None):
    rclpy.init(args=args)
    drive_publisher = DrivePublisher()
    rclpy.spin(drive_publisher)
    drive_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()