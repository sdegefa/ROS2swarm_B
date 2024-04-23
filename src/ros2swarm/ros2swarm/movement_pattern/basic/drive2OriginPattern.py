#    Copyright 2023 Solan Degefa
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Vector3
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils import setup_node
from math import atan2, pi, cos, sin
from icecream import ic
# import sys
# ic(sys.path)
class Drive2OriginPattern(MovementPattern):
    """
    A simple pattern for driving a constant direction vector.

    Which is configured in with the parameters of this pattern.
    How often the direction is published is configured in the timer period parameter.
    """

    def __init__(self):
        """Initialize the drive pattern."""
        super().__init__('drive2OriginPattern')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('drive_timer_period', 0.05),
                ('speed', 0.1),
                ('leaker_range', 1.0),
                
            ])
        
        # robot_X/odom topic subscriber
        # Used to gain positional and rotational data relative to 0,0,0 
        self.tf_data_subscription = self.create_subscription(
             Odometry,
             self.get_namespace() + '/odom',
             self.swarm_command_controlled(self.odom_callback), 
            10
        )
           
        self.position = {"x":0, "y":0, "z":0, "yaw":0.0, "pitch":0.0}
        self.namespace = self.get_namespace()
        # ic(self.namespace)
        # ic(self.position)
        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        self.i = 0
        self.param_Speed = float(self.get_parameter("speed").get_parameter_value().double_value)
        self.param_LRange = float(
            self.get_parameter("leaker_range").get_parameter_value().double_value)

        self.get_logger().warn('Logger is: ' + self.get_logger().get_effective_level().name)
        self.get_logger().info(f'Speed is: {self.param_Speed}')
        self.get_logger().info(f'Leaker Range is: {self.param_LRange}')
        self.get_logger().debug('Logger is: debug')
        # self.angularVector3= Vector3(x=0.0,y=0.0,z=0.0) # default angular velocities so the timer doesnt get mad

        self.startingPosition = []
        ic(self.startingPosition)
    
    
    def odom_callback(self, odom_msg):
        """ When called, will use odometry data to set drive velocities towards the origin """
        
        """
        JUST UPDATE ANGULAR VELOCITY. SET X VELOCITY TO A CONSTANT SPEED ( WILL HAVE ROBOT/DRONE DRIVE DIRECTION IT IS FACING ) 
        FIND X VELOCITY THAT MAKES SENSE, ROS2SWARM CREATORS HINT VELOCITY IS IN M/S        
        """
        self.orientation_update(odom_msg)
        # msg = Twist()
        # ic(yaw_correction)
        # yaw_correction =   (atan2(self.position['y'], self.position['x'])+ pi - self.position["yaw"]) % (2 * pi)           
        # if yaw_correction < -pi:
        #    yaw_correction += 2 * pi
        # elif yaw_correction >= pi:
        #    yaw_correction -= 2 * pi
        # msg.angular.z = yaw_correction
         
            #  msg.linear.x = 1.0

            #  self.get_logger().info('Publishing {}:"{}"'.format(self.i, msg))
            #  self.command_publisher.publish(msg)

    def orientation_update(self, odom_msg)-> None:
        if odom_msg is None:
            return 0
        # ic(odom_msg)
        
        # Pulling Base Link Translation and Rotation data
        positionalData = odom_msg.pose.pose.position
        self.position['x'] = positionalData.x
        self.position['y'] = positionalData.y
        self.position['z'] = positionalData.z
        
        self.position["yaw"] = quaternion_to_yaw(odom_msg.pose.pose.orientation)
        # ic(self.position["yaw"])
        
        if not self.startingPosition:
            self.startingPosition = [
                self.position['x'],
                self.position['y'],
                self.position['z']
            ] 

        # yaw = (atan2(self.position['y'], self.position['x'])+pi)
        
        # ic(yaw)
        # return abs(yaw - 2*pi) - abs(self.position["yaw"])

    def timer_callback(self):
        """Uses currently updates location to ."""
        if abs(self.position['x']) < self.param_LRange and abs(self.position['y']) < self.param_LRange:
            ic("Robot Has Leaked, Commence Deletion")
            return
        
        msg = Twist()
        # command to publish the message in the terminal by hand
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        # linear: {x: 0.26, y: 0.0, z: 0.0},
        # angular: {x: 0.0, y: 0.0, z: 0.0}
        # }"

        # # using a little bit of trig, I am able to calculate the angle necessary to make the drone face the origin, 
        # # that way, when giving the drone a velocity, it will travel directly over the origin line.
        
        ic("---------------------------------------------------------------")
        yaw_correction =   (atan2(self.position['y'], self.position['x'])+ pi - self.position["yaw"]) % (2 * pi)
        
        if yaw_correction < -pi:
            yaw_correction += 2 * pi
        elif yaw_correction >= pi:
            yaw_correction -= 2 * pi

        msg.angular.z = yaw_correction
        msg.linear.x = self.param_Speed
        
        self.get_logger().info('Publishing {}:"{}"'.format(self.i, msg))
        self.command_publisher.publish(msg)

        self.i += 1
        self.namespace


def calculate_Vector(yaw:float, pitch: float = 0.0, roll: float = 0.0) -> Vector3:
    angularVector3 = Vector3()
    t0 = cos(yaw * 0.5)
    t1 = sin(yaw * 0.5)
    t2 = cos(roll * 0.5) # Roll is 0 for the time being, do not forsee a situation where this is nessessary yet - 1/30/24
    t3 = sin(roll * 0.5) # Roll is 0 for the time being, do not forsee a situation where this is nessessary yet - 1/30/24
    t4 = cos(pitch * 0.5)
    t5 = sin(pitch * 0.5)
    
    angularVector3.x = t0 * t3 * t4 - t1 * t2 * t5
    angularVector3.y = t0 * t2 * t5 + t1 * t3 * t4
    angularVector3.z = t1 * t2 * t4 - t0 * t3 * t5
    
    return angularVector3

def quaternion_to_yaw(quaternion):
    """Calculate yaw angle from a quaternion.

    Args:
    - q (list): A list representing the quaternion in the order [w, x, y, z].

    Returns:
    - float: Yaw angle in radians.
    
    From ChatGPT
    """
    # Extract quaternion components
    w = quaternion.w 
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z 
    # Yaw (z-axis rotation)
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))





def main(args=None):
    """
    Create a node for the drive pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, Drive2OriginPattern)


if __name__ == '__main__':
    main()
