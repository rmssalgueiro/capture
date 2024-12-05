import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import Waypoint, AddCircle, SetMode
from pegasus_msgs.msg import AutopilotStatus
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from capture_msgs.msg import Capture

import sys


class Drone(Node):

    def __init__(self, id):
        super().__init__('drone_api_' + str(id))

        self.id = id
        self.namespace = 'drone'
        self.current_position = (0.0, 0.0, 0.0)  # Initialize current position

        # Variables to store the initial position
        self.initial_position_received = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_z = 0.0

        # Create the service clients for the drone
        self.add_waypoint_srv = self.create_client(Waypoint, '/drone' + str(id) + '/autopilot/set_waypoint')
        while not self.add_waypoint_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waypoint service not available, waiting again...')

        self.set_autopilot_srv = self.create_client(SetMode, '/drone' + str(id) + '/autopilot/change_mode')
        while not self.set_autopilot_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Set Mode service not available, waiting again...')

        # Create subscriptions to listen to the drone's position (replace PositionStatus with your message type)
        self.create_subscription(Capture, '/drone2/capture/status', self.position_status_callback, qos_profile_sensor_data)

        # Requests messages
        self.waypoint_req = Waypoint.Request()
        self.set_mode_req = SetMode.Request()

    def position_status_callback(self, msg):
        self.current_position = (
            msg.global_pos_target[0],
            msg.global_pos_target[1],
            msg.global_pos_target[2]
        )

    def set_autopilot_mode(self, mode='DisarmMode'):
        self.get_logger().info(f'Setting autopilot mode to: {mode}')
        self.set_mode_req.mode = mode
        self.future = self.set_autopilot_srv.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_waypoint(self, x, y, z, yaw):
        self.get_logger().info(f'Setting waypoint to: {x}, {y}, {z}, {yaw}')
        self.waypoint_req.position[0] = x
        self.waypoint_req.position[1] = y
        self.waypoint_req.position[2] = z
        self.waypoint_req.yaw = yaw
        self.future = self.add_waypoint_srv.call_async(self.waypoint_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    #shuttle = Drone(1)
    target = Drone(2)


    # Arm the drone
    #shuttle.set_autopilot_mode('ArmMode')
    #shuttle.set_autopilot_mode('TakeoffMode')
    target.set_autopilot_mode('ArmMode')
    target.set_autopilot_mode('TakeoffMode')

    # Wait for takeoff
    time.sleep(5)

    waypoints = [
            (0.0, 0.0, -10.0),
            (-67.35, 52.7, -10.0),
            (107, -14.5, -10.0),
            (0.0, 10.0, -10.0)
            
        ]

        # Set waypoints one by one
    for i, waypoint in enumerate(waypoints):
        # Calculate yaw only if there is a next waypoint
        if i < len(waypoints) - 1:
            next_waypoint = waypoints[i + 1]
            dx = next_waypoint[0] - waypoint[0]
            dy = next_waypoint[1] - waypoint[1]
            yaw = math.atan2(dy, dx)  # Yaw towards the next waypoint
        else:
            yaw = 0.0  # Final waypoint: keep a default yaw

        # Set the waypoint with the calculated yaw
        target.set_waypoint(*waypoint, yaw=yaw)
        target.set_autopilot_mode('CoordenadasMode')

        # Wait until the drone reaches the current waypoint within a 1-meter margin
        while True:
            rclpy.spin_once(target)
            distance = math.sqrt(
                (target.current_position[0] - waypoint[0])**2 +
                (target.current_position[1] - waypoint[1])**2 +
                (target.current_position[2] - waypoint[2])**2
            )
            if distance <= 2.0:
                target.get_logger().info(f"Reached waypoint at: {waypoint}")
                break  # Move to the next waypoint

    target.get_logger().info("All waypoints reached.")
    
    # Set waypoints relative to the initial position
    #shuttle.set_waypoint(initial_x, initial_y, initial_z - 3.0, 0.0)
    #shuttle.set_waypoint(3.0 , 3.0 , -3.0, 1.0)
    #target.set_waypoint(100, 10.0, -20.0, 2.0)
    
    #shuttle.set_autopilot_mode('CoordenadasMode')
    #target.set_autopilot_mode('CoordenadasMode')

    #time.sleep(10)
    '''
    shuttle.set_waypoint(0.0, 3.0 , -3.0, 1.0)
    target.set_waypoint(3.0, 0.0 , -3.0, 2.0)
    #shuttle.set_waypoint(10, 0, -10.0, 2.0)
    
    time.sleep(10)
    
    shuttle.set_waypoint(0.0, 0.0 , -3.0, 1.0)
    target.set_waypoint(3.0, 3.0 , -3.0, 2.0)
    #shuttle.set_waypoint(10, 10, -10.0, 0.0)
    
    time.sleep(10)
    
    shuttle.set_waypoint(3.0, 0.0 , -3.0, 1.0)
    target.set_waypoint(0.0, 3.0 , -3.0, 2.0)
    
    time.sleep(10)

    shuttle.set_waypoint(3.0, 3.0 , -3.0, 1.0)
    target.set_waypoint(0.0, 0.0 , -3.0, 2.0)

    time.sleep(10)
    '''
    '''
    shuttle.set_waypoint(initial_x + 3.0, initial_y + 3.0, initial_z - 3.0, 0.0)

    time.sleep(5)
    shuttle.set_waypoint(initial_x, initial_y + 3.0, initial_z - 3.0, 0.0)

    time.sleep(5)
    shuttle.set_waypoint(initial_x, initial_y, initial_z - 3.0, 0.0)

    time.sleep(5)
    '''
    # Land the drone
    #shuttle.set_autopilot_mode('OnboardLandMode')
    target.set_autopilot_mode('OnboardLandMode')
    time.sleep(5)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
