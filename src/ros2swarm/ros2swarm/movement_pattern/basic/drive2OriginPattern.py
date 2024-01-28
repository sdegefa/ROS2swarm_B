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
from geometry_msgs.msg import Twist
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ros2swarm.utils import setup_node
from icecream import ic


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
                ('drive_timer_period', 0.5),
                ('targetX', 0.0),
                ('targetY', 0.0),
            ])

        self.namespace = self.get_namespace()
        ic(self.namespace)
        timer_period = float(
            self.get_parameter("drive_timer_period").get_parameter_value().double_value)
        self.timer = self.create_timer(timer_period, self.swarm_command_controlled_timer(self.timer_callback))
        self.i = 0
        self.param_x = float(self.get_parameter("targetX").get_parameter_value().double_value)
        self.param_z = float(
            self.get_parameter("targetY").get_parameter_value().double_value)

        self.get_logger().warn('Logger is: ' + self.get_logger().get_effective_level().name)
        self.get_logger().info('Logger is: Not  but it did update Tho ')
        self.get_logger().info('This Shit working good')
        self.get_logger().debug('Logger is: debug')

    def timer_callback(self):
        """Publish the configured twist message when called."""
        
        msg = Twist()
        # command to publish the message in the terminal by hand
        # ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
        # linear: {x: 0.26, y: 0.0, z: 0.0},
        # angular: {x: 0.0, y: 0.0, z: 0.0}
        # }"
       
        msg.linear.x = self.param_x
        msg.angular.z = self.param_z
        self.command_publisher.publish(msg)
        self.get_logger().debug('Publishing {}:"{}"'.format(self.i, msg))
        self.get_logger().info(f'We out here | Robot {self.namespace}')
        self.i += 1

        self.namespace



def main(args=None):
    """
    Create a node for the drive pattern, spin it
    and handle the setup.
    """
    setup_node.init_and_spin(args, Drive2OriginPattern)


if __name__ == '__main__':
    main()
