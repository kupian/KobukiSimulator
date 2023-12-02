import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
from ugrdv_kobuki_msgs.msg import DriveCommand
import math

### This node subscribes to the /cone_positions topic provided by Unity
### and uses a simple test algorithm to publish a drive command.

class Cone():
    def __init__(self, x, y, distance, colour):
        self.x = x
        self.y = y
        self.distance = distance
        self.colour = colour

    def __str__(self):
        return f"{self.x},{self.y},{self.distance}m,{self.colour}"

class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.cone_pos_subscriber = self.create_subscription(
            String,
            '/cone_positions',
            self.cone_pos_subscriber_callback,
            1
        )

        self.drive_publisher = self.create_publisher(DriveCommand, "/ugrdv_kobuki/drive_command", 1)
        self.cone_pos_subscriber
        self.visibleCones = []

        self.linearSpeed = 1.0
        self.angularSpeed = 0.5

        self.SetRotationSpeed(0.0)

    def cone_pos_subscriber_callback(self, msg):
        self.visibleCones = []
        cones = msg.data.split(")") # Split cones by right bracket
        self.CreateConeFromMsg(cones)
        desiredRotationSpeed = self.GetDesiredRotationSpeed()
        if desiredRotationSpeed:
            self.SetRotationSpeed(desiredRotationSpeed)
    
    def GetNearestCone(self, colour):

        if len(self.visibleCones) < 1:
            return None
        
        closest_cone = None
        min_distance = float('inf')  # Set initial minimum distance to positive infinity

        for cone in self.visibleCones:
            if cone.colour == colour:
                distance = cone.distance

                if distance < min_distance:
                    min_distance = distance
                    closest_cone = cone

        return closest_cone
    
    def GetMidpoint(self, blueCone,yellowCone):
        midpoint = None
        if blueCone and yellowCone:
            midpoint = ((blueCone.x+yellowCone.x)/2, (blueCone.distance+yellowCone.distance)/2)
        return midpoint
    
    def GetAngleToPoint(self, x,y):
        angle = math.atan(y/x)
        return angle
    
    def CreateConeFromMsg(self, coneString): # Convert msg into cone object and add to list
        for cone in coneString:
            if len(cone) < 1:
                continue
            cone = cone.split("(")[1].split(",") # Split into list of values
            x = float(cone[0])
            y = float(cone[1])
            distance = float(cone[2])
            colour = "blue" if float(cone[3]) < 1 else "yellow"
            self.visibleCones.append(Cone(x,y,distance,colour))

    def GetDesiredRotationSpeed(self):
        nearestMidpoint = self.GetMidpoint(self.GetNearestCone("blue"),self.GetNearestCone("yellow"))
        if nearestMidpoint:
            self.get_logger().info(f"midpoint offset: {nearestMidpoint}")
            if nearestMidpoint[0] > 0.5:
                return 1.0 * self.angularSpeed
            else:
                return -1.0 *self.angularSpeed
    
    def SetRotationSpeed(self, speed):
        drive_command = DriveCommand()
        drive_command.linearvel = self.linearSpeed
        drive_command.angularvel = speed
        self.drive_publisher.publish(drive_command)

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
