import rclpy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import csv
import numpy as np
import math
import time
class MultiGoals(Node):
    def __init__(self, goals, retry, map_frame):
        super().__init__('multi_goals')
        self.get_logger().info("Initializing MultiGoals node...")
       
        self.retry = retry
        if self.retry == 1:
            self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.pose_ekf_sub = self.create_subscription(Odometry, '/encoder_imu_odom', self.get_pose_ekf, 10)
            self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
            self.pub_final = self.create_publisher(Float64, '/arrfinal', 1)
            
            self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.tf_buffer = tf2_ros.Buffer()
            self.sub_tf = tf2_ros.TransformListener(self.tf_buffer,self)
            self.current_goal_handle = None
            self.goals = goals
            self.current_goal_index = 1
            self.MIN_DISTANCE = 1.3
            self.goal_received = False
            self.goalMsg = PoseStamped()
            self.goalMsg.header.frame_id = map_frame
            self.count = 1
            self.concel_goal_count = 0
            self.timer = self.create_timer(2, self.publish_goal)
            self.get_logger().info("Initializing node...")
        self.get_logger().info("Initialize finish...")
        

    def publish_goal(self):
        # if (not self.goal_received and self.current_goal_index <= len(self.goals) ) or (not self.current_goal_handle.accepted):
        if not self.goal_received and self.current_goal_index <= len(self.goals):
            goal = self.goals[self.current_goal_index - 1]
            self.goalMsg.header.stamp = self.get_clock().now().to_msg()
            self.goalMsg.pose.position.x = goal[0]
            self.goalMsg.pose.position.y = goal[1]
            self.goalMsg.pose.orientation.z = goal[2]
            self.goalMsg.pose.orientation.w = goal[3]
            self.pub.publish(self.goalMsg)

            navigate_goal = NavigateToPose.Goal()
            navigate_goal.pose = self.goalMsg

            self.action_client.wait_for_server()
            send_goal_future = self.action_client.send_goal_async(navigate_goal)
            send_goal_future.add_done_callback(self.goal_response_callback)

            self.get_logger().info(f"Published goal pose with ID: {self.current_goal_index}")
        
            

    def goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        self.get_logger().info(f"Published goal result: {self.current_goal_handle.accepted}")
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.pub.publish(self.goalMsg)
            navigate_goal = NavigateToPose.Goal()
            navigate_goal.pose = self.goalMsg

            self.action_client.wait_for_server()
            send_goal_future = self.action_client.send_goal_async(navigate_goal)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info("Published again")
            return

        self.get_logger().info('Goal accepted')
        # self.current_goal_handle.get_result_async().add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result}")

    def plan_callback(self, msg):
        if msg.poses:
            self.goal_received = True
            self.get_logger().info('Received valid plan')
            self.timer.cancel()

    def cancel_goal(self):
        self.get_logger().info("Cancelling current goal")
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if cancel_response is not None and cancel_response.return_code == 0:  # 0 indicates success
            self.get_logger().info("Goal cancellation accepted")
            self.concel_goal_count = 0
        else:
            self.get_logger().info("Goal cancellation failed")
            self.concel_goal_count += 1 
            if self.concel_goal_count < 5:
                self.cancel_goal()

    def status_cb(self):
        if self.current_goal_index > len(self.goals):
            self.get_logger().info("All goals processed.")
            return
        # self.get_logger().info("start tf_transform")

        current_goal = self.goals[self.current_goal_index - 1]
        point_source = PointStamped()
        point_source.header.frame_id = "map"
        point_source.header.stamp = rclpy.time.Time().to_msg()
        point_source.point.x = current_goal[0]
        point_source.point.y = current_goal[1]
        point_source.point.z = 0.0
        # self.get_logger().info(f"point_source.x: {point_source.point.x}")
        # self.get_logger().info(f"point_source.y: {point_source.point.y}")

        #point_target = self.tf_buffer.transform(point_source,"odom",rclpy.duration(1))
        try:
          point_target = self.tf_buffer.transform(point_source, "odom",rclpy.duration.Duration(seconds=1.0))
          #point_target = point_source
        #   self.get_logger().info(f"point_source.y: {point_target.point.y}")
        except:
          self.get_logger().info("lookup tf error")
          point_target = point_source
        distance_to_goal = self.distance(self.kx, self.ky, point_target.point.x, point_target.point.y)

        self.get_logger().info(f"Distance to goal: {distance_to_goal}")

        if distance_to_goal < self.MIN_DISTANCE:
            self.cancel_goal()
            self.goal_received = False
            self.current_goal_index += 1
            if self.current_goal_index <= len(self.goals):
                
                self.publish_goal()
            else:
                self.concel_goal_count = 5
                self.get_logger().info("Final goal reached; all goals completed.")

    def get_pose_ekf(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        self.count = 1+self.count
        if self.count > 500 :
            self.status_cb()
            self.count = 1

    def distance(self, kx, ky, gx, gy):
        return math.sqrt((kx - gx) ** 2 + (ky - gy) ** 2)

def main(args=None):
    rclpy.init(args=args)

    with open('test.csv', 'r') as f:
        reader = csv.reader(f)
        goals = np.array([list(map(float, cols)) for cols in reader])
        print("Multi Goals Executing...")

    if goals.shape[1] == 4 and goals.shape[0] > 0:
        print("Multi Goals Executing...")
        mg = MultiGoals(goals, 1, 'map')
        rclpy.spin(mg)
        mg.destroy_node()
    else:
        print("Invalid goal list format")

    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
