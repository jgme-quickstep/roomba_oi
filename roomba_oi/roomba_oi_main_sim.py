"""Simulated Roomba bridge node.
This node serves as the simulation interface for the Roomba.

ROS logic ==========================================
Pub: roomba_state
Sub: roomba_commands
Gazebo logic =======================================
Sub: cmd_vel
Pub: model/quickstep/wheel_angles
"""

import math
import rclpy
from rclpy.node import Node
from enum import Enum
from roomba_bridge_messages.msg import RoombaState, RoombaCommands, RoombaElectrical, OdomData
from geometry_msgs.msg import Twist, Vector3

class OIState(Enum):
    """Encapsulates the simulated state of the Roomba OI service"""
    OFF = 0
    PASSIVE = 1
    SAFE = 2
    FULL = 3

class RoombaBridge(Node):
    """Roomba bridge node for use in Gazebo"""
    def __init__(self):
        """
            Setup ROS components and Roomba adapter. Designed for use with Gazebo sim
        """
        super().__init__('roomba_bridge')

        # Serial port for roomba
        self.port = "/dev/ttyUSB0"

        # Roomba state
        self.oi_state: OIState = OIState.OFF
        self.change_oi_state(OIState.FULL)

        # Setup subscriber for commands
        self.subscription = self.create_subscription(
            RoombaCommands,
            'roomba_commands',
            self.command_callback,
            10
        )

        # Setup subscriber for wheel angles, state is published based on this callback
        self.subscription = self.create_subscription(
            Vector3,
            'model/quickstep/wheel_angles',
            self.odom_callback_publish_state,
            10
        )
        
        self.publisher_state = self.create_publisher(RoombaState, 'roomba_state', 10) # Setup the state publisher
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10) # Setup the state publisher

        self.OdomData = OdomData() # Initialize odom data so that we can use it in the callback


    # Handle commands from the roomba_commands topic
    def command_callback(self, msg: Twist):
        """
            Handles writing requests in the RoombaCommands topic to the roomba.
        """

        # Linear and angular velocity request in roomba frame
        linear_cmd = msg.cmd_vel.linear.x
        angular_cmd = msg.cmd_vel.angular.z

        # Send wheel commands to roomba (Gazebo bridge)
        msg_cmd_vel = Twist()

        msg_cmd_vel.linear.x = linear_cmd
        msg_cmd_vel.angular.z = angular_cmd

        self.publisher_cmd_vel.publish(msg_cmd_vel)

    def odom_callback_publish_state(self, msg: Vector3) -> None:
        """
            Callback on odometer data, also publishes the RoombaState topic
        """
        self.get_logger().debug("Entering odom callback")
        msg_odom = OdomData()
        count_per_rev = 508.8 # Wheel encoder counter per revolution (per OI spec)
        # Convert from angle to encoder count
        msg_odom.left_wheel_encoder = int(count_per_rev * (msg.x / (2*math.pi)))
        msg_odom.right_wheel_encoder = int(count_per_rev * (msg.y / (2*math.pi)))

        delta_left_encoder = float(msg_odom.left_wheel_encoder - self.OdomData.left_wheel_encoder)
        delta_right_encoder = float(msg_odom.right_wheel_encoder - self.OdomData.right_wheel_encoder)

        # Odom message from gz provides wheel angles
        msg_odom.left_wheel_angle = msg.x
        msg_odom.right_wheel_angle = msg.y

        # Distance is the average of both wheel displacements since the last message
        msg_odom.distance_delta = (math.pi * (72 / count_per_rev)) * (delta_left_encoder + delta_right_encoder) / 2
        wheel_base_distance =  0.219 # wheel separation, meters
        msg_odom.angle_delta = (math.pi * (72 / count_per_rev)) * (delta_right_encoder - delta_right_encoder) / wheel_base_distance

        ## Publish state
        state_msg_to_pub = self.fetch_state()   # Generate fake signals 
        state_msg_to_pub.odom_data = msg_odom    # Load in gazebo odometer data
        self.OdomData = msg_odom
        self.publisher_state.publish(state_msg_to_pub)

    # Handle the state update topic
    def fetch_state(self) -> RoombaState:
        """
            Provides a mostly complete Roomba state message, by faking required parameters
        """
        msg = RoombaState();
        msg.connected = True # Always connected in simulation
        msg.oi_state = self.oi_state.value
        msg.port = self.port
        msg.batt_state = self.fetch_battery() # Grab the battery data

        return msg

    def fetch_battery(self) -> RoombaElectrical:
        """
            Fetches the roomba battery state and returns a RoombaElectrical message 
            containing the data.
        """
        msg = RoombaElectrical()
        msg.bat_volt_mv = 30000 # TODO: make this a real function of usage
        msg.bat_amp_ma = 1000   # TODO: make this a real function of usage
        msg.bat_temp_c = 20     # Let this be a constant
        msg.bat_capacity = 20000# Let this be a constant

        return msg
    
    def change_oi_state(self, req_oi_state: OIState):
        """
            Changes the OI state, used for simulation the mock Roomba state
        """
        # Updates the OI state with the requested OI state (for simulation)
        self.oi_state = req_oi_state

def main(args=None):
    rclpy.init(args=args)

    roomba_bridge = RoombaBridge()
    rclpy.spin(roomba_bridge)

    roomba_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()