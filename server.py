
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mcp.server.fastmcp import FastMCP
import asyncio
import threading

class ROS2MCPNode(Node):
    def __init__(self):
        super().__init__('ros2_mcp_server_node')

        # pub / sub
        self.pub_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info('ROS2 node initialized')

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

def run_ros2_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    ros_node = ROS2MCPNode()

    ros_thread = threading.Thread(target=run_ros2_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    mcp = FastMCP("ros2-mcp-server")
    @mcp.tool()
    async def move_robot(speed: float, angle: float, duration: float) -> str:
        return await ros_node.send_twist_with_duration(
            linear=[speed, 0.0, 0.0], 
            angular=[0.0, 0.0, angle],
            duration=duration
        )

    mcp.run(transport="stdio")

    # clean up
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


