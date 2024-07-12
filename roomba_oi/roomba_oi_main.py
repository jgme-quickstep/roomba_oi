"""Bridge node for controlling a physical Roomba.
This node serves as the interface for the Roomba.

ROS logic ==========================================
Pub: roomba_state
Sub: roomba_commands
"""

import math
import rclpy
from rclpy.node import Node
from pyroombaadapter import PyRoombaAdapter
from roomba_bridge_messages.msg import RoombaState, RoombaCommands, RoombaElectrical, OdomData
from geometry_msgs.msg import Twist

class RoombaBridge(Node):
    def __init__(self):
        """
            Setup ROS components and Roomba adapter.
        """
        super().__init__('roomba_bridge')

        # Serial port for roomba
        self.port = "/dev/ttyUSB0"
        try:
            self.roomba = PyRoombaAdapter(self.port)
            self.roomba.change_mode_to_full()
            self.get_logger().debug('Connection to roomba established')
        except Exception as e:
            self.roomba = None
            self.get_logger().debug('Serial connection to roomba failed')

        # Setup subscriber for commands
        self.subscription = self.create_subscription(
            RoombaCommands,
            'roomba_commands',
            self.command_callback,
            10
        )

        timer_period = 0.1
        self.state_update_timer =  self.create_timer(timer_period, self.state_update_callback) # Call publisher every x seconds
        self.publisher_ = self.create_publisher(RoombaState, 'roomba_state', 10) # Setup the state publisher

    # Handle commands from the roomba_commands topic
    def command_callback(self, msg: RoombaCommands):
        """
            Handles writing requests in the RoombaCommands topic to the roomba.
        """

        # Linear and angular velocity request in roomba frame
        linear_cmd = msg.cmd_vel.linear.x
        angular_cmd = msg.cmd_vel.angular.z

        # Send wheel commands to roomba
        self.roomba.move(linear_cmd, angular_cmd)

    # Handle the state update topic
    def state_update_callback(self):
        """
            Polls the roomba for sensor data and publishes a RoombaState message to 
            the roomba_state topic.
        """
        msg = RoombaState();
        msg.connected = self.roomba is not None
        msg.oi_state = self.roomba.request_oi_mode() # OI mode data
        msg.port = self.port
        msg.batt_state = self.fetch_battery() # Grab the battery data
        msg.odom_data = self.fetch_encoder_odom() # Grab the encoder odom data

        self.publisher_.publish(msg)

    def fetch_battery(self) -> RoombaElectrical:
        """
            Fetches the roomba battery state and returns a RoombaElectrical message 
            containing the data.
        """
        msg = RoombaElectrical() # Empty electrical message
        msg.bat_volt_mv = self.roomba.request_voltage()
        msg.bat_amp_ma = self.roomba.request_current()
        msg.bat_temp_c = self.roomba.request_temperature()
        msg.bat_capacity = self.roomba.request_charge()

        return msg
    
    def fetch_encoder_odom(self) -> OdomData:
        """
            Fetches the roomba odom encoder data, returning an OdomData message.
        """
        msg = OdomData() # Empty encoder message

        msg.distance_delta = self.roomba.request_distance()
        msg.angle_delta = self.roomba.request_angle()

        # Raw encoder data
        encoder_counts = self.roomba.request_encoder_counts()
        # Encoder counts is a tuple of form (left count, right count)
        msg.left_wheel_encoder = int(encoder_counts[0])
        msg.right_wheel_encoder = int(encoder_counts[1])

        count_per_rev = 508.8 # Wheel encoder counter per revolution (per OI spec)
        msg.left_wheel_angle = float(encoder_counts[0]/count_per_rev * 2.0 * math.pi)
        msg.right_wheel_angle = float(encoder_counts[1]/count_per_rev * 2.0 * math.pi)

        msg.distance_delta = 0.0
        msg.angle_delta = 0.0

        return msg

def main(args=None):
    rclpy.init(args=args)

    roomba_bridge = RoombaBridge()
    rclpy.spin(roomba_bridge)

    roomba_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()