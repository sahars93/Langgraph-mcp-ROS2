import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from mcp.server.fastmcp import FastMCP
import asyncio
import threading
import os
import re
import subprocess
import time
from typing import List, Optional, Tuple



class ROS2MCPNode(Node):
    def __init__(self):
        super().__init__('ros2_mcp_server_node')

        # pub / sub
        self.pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.pose_data = None
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('ROS2 node initialized')
        
    def pose_callback(self, msg):
        self.pose_data = msg
        
    async def get_position(self) -> dict:
        if self.pose_data is None:
            self.get_logger().warn('pose data not available')
            return {
                "position": None,
                "error": "pose data not available"
            }
        
        try:
            position = {
                "x": float(self.pose_data.x),
                "y": float(self.pose_data.y)
                                        }
            
            
            return {
                "position": position,
                "linear_velocity": {
                    "x": float(self.pose_data.linear_velocity)
                },
                "angular_velocity": {
                    "x": float(self.pose_data.theta)
                }
            }
        except Exception as e:
            self.get_logger().error(f"Error getting position: {str(e)}")
            return {
                "position": None,
                "error": str(e)
            }

    async def send_twist_with_duration(self, linear: list[float], angular: list[float], duration: float) -> str:
        try:
            msg = Twist()
            msg.linear.x = float(linear[0])
            msg.linear.y = float(linear[1]) if len(linear) > 1 else 0.0
            msg.linear.z = float(linear[2]) if len(linear) > 2 else 0.0
            msg.angular.x = float(angular[0]) if len(angular) > 0 else 0.0
            msg.angular.y = float(angular[1]) if len(angular) > 1 else 0.0
            msg.angular.z = float(angular[2]) if len(angular) > 2 else 0.0

            start_time = self.get_clock().now()
            duration_sec = duration
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration_sec:
                self.pub_cmd_vel.publish(msg)
                self.get_logger().info(f'Publishing Twist: linear={msg.linear.x}, angular={msg.angular.z}')
                await asyncio.sleep(0.1)  # 10Hz

            stop_msg = Twist()
            self.pub_cmd_vel.publish(stop_msg)
            self.get_logger().info('Published stop command')
            return f"Successfully moved for {duration} seconds and stopped"
        except Exception as e:
            self.get_logger().error(f"Error publishing Twist: {str(e)}")
            return f"Error: {str(e)}"



def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")
    valid_ros2_commands = ["node", "topic", "service", "param", "doctor"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)

def get_entities(
    cmd: str,
    delimiter: str = "\n",
    pattern: str = None,
    blacklist: Optional[List[str]] = None,
) -> List[str]:
    """
    Get a list of ROS2 entities (nodes, topics, services, etc.).

    :param cmd: the ROS2 command to execute.
    :param delimiter: The delimiter to split the output by.
    :param pattern: A regular expression pattern to filter the list of entities.
    :return:
    """
    success, output = execute_ros_command(cmd)

    if not success:
        return [output]

    entities = output.split(delimiter)

    # Filter out blacklisted entities
    if blacklist:
        entities = list(
            filter(
                lambda x: not any(
                    re.match(f".*{pattern}.*", x) for pattern in blacklist
                ),
                entities,
            )
        )

    if pattern:
        entities = list(filter(lambda x: re.match(f".*{pattern}.*", x), entities))

    entities = [e for e in entities if e.strip() != ""]

    return entities


def run_ros2_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    ros_node = ROS2MCPNode()

    ros_thread = threading.Thread(target=run_ros2_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    mcp = FastMCP("ros2-mcp-server")

    @mcp.tool()
    async def move_robot(linear: list[float], angular: list[float], duration: float) -> str:
        """Send movement commands to the robot for a specified duration"""
        return await ros_node.send_twist_with_duration(linear, angular, duration)
        
    @mcp.tool()
    async def get_position() -> dict:
        """Get the current position of the robot"""
        return await ros_node.get_position()


    @mcp.tool()
    async def ros2_topic_list(pattern: Optional[str] = None, blacklist: Optional[List[str]] = None) -> dict:
        """
        Get a list of ROS2 topics.

        :param pattern: A regular expression pattern to filter the list of topics.
        """
        cmd = "ros2 topic list"
        topics = get_entities(cmd, pattern=pattern, blacklist=blacklist)
        return {"topics": topics}



    mcp.run()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()