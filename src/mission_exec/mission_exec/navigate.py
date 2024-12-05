# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator(namespace="Turtlebot_02490")
    # self.get_logger().info("Starting the navigator... ",navigator)
    # Start on dock
    # if not navigator.getDockedStatus():
    #     navigator.info('Docking before intialising pose')
    #     navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([-0.343, 0.294], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2

    navigator.waitUntilNav2Active()

    # # Set goal poses
    # goal_pose = navigator.getPoseStamped([-3.5,0.0], TurtleBot4Directions.EAST)

    # # Undock
    # navigator.undock()

    # # Go to each goal pose
    # navigator.startToPose(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()