# Subscribe to joystick command and adapt it to a Toomba command
import rclpy
from rclpy.node import Node
from roomba_bridge_messages.msg import RoombaCommands
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class RoombaTeleop(Node):
    def __init__(self):
        """
            Setup ROS components and Roomba adapter.
        """
        super().__init__('roomba_bridge')

        # Setup subscriber for commands
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.publisher_gen_ = self.create_publisher(RoombaCommands, 'roomba_commands', 10) # Setup the general command publisher

    # Handle commands from the roomba_commands topic
    def joy_callback(self, msg: Joy):
        """
            Reads in a joystick command and republishes it as a wheelspeed command
        """
        scale = 1 # Temporary scaling factor, for development
        msg_tw = Twist()
        msg_tw.linear.x = scale * msg.axes[1]
        msg_tw.angular.z = scale * msg.axes[3]
        
        msg_cmd = RoombaCommands()
        msg_cmd.cmd_vel = msg_tw

        self.publisher_gen_.publish(msg_cmd)
        

def main(args=None):
    rclpy.init(args=args)

    roomba_bridge = RoombaTeleop()
    rclpy.spin(roomba_bridge)

    roomba_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()