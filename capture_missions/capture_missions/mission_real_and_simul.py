import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import Waypoint, AddCircle, SetMode
from pegasus_msgs.msg import AutopilotStatus
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import sys


class Drone(Node):

    def __init__(self, id):
        super().__init__('drone_api_' + str(id))

        self.id = id
        self.namespace = 'drone'

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
        self.create_subscription(Odometry, '/drone' + str(id) + '/fmu/filter/state', self.position_status_callback, qos_profile_sensor_data)

        # Requests messages
        self.waypoint_req = Waypoint.Request()
        self.set_mode_req = SetMode.Request()

    def position_status_callback(self, msg):
        if not self.initial_position_received:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.initial_z = msg.pose.pose.position.z
            self.initial_position_received = True
            self.get_logger().info(f'Initial position saved: ({self.initial_x}, {self.initial_y}, {self.initial_z})')

    def set_autopilot_mode(self, mode='DisarmMode'):
        self.get_logger().info(f'Setting autopilot mode to: {mode}')
        self.set_mode_req.mode = mode
        self.future = self.set_autopilot_srv.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def set_waypoint(self, x, y, z, yaw):
        #self.get_logger().info(f'Setting waypoint to: {x}, {y}, {z}, {yaw}')
        self.waypoint_req.position[0] = x
        self.waypoint_req.position[1] = y
        self.waypoint_req.position[2] = z
        self.waypoint_req.yaw = yaw
        self.future = self.add_waypoint_srv.call_async(self.waypoint_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    real = Drone(1)
    simul = Drone(2)

    # Wait until initial position is received
    while not real.initial_position_received:
        rclpy.spin_once(real)

    # Now use the initial position to set waypoints dynamically
    initial_x = real.initial_x
    initial_y = real.initial_y
    initial_z = real.initial_z

    if initial_x == 0.0:
        sys.exit(0)

    # Arm the drone
    real.set_autopilot_mode('ArmMode')
    real.set_autopilot_mode('TakeoffMode')
    
    simul.set_autopilot_mode('ArmMode')
    simul.set_autopilot_mode('TakeoffMode')

    # Wait for takeoff
    time.sleep(5)

    '''
    # Set waypoints relative to the initial position
    shuttle.set_waypoint(initial_x, initial_y, initial_z - 3.0, 0.0)
    shuttle.set_autopilot_mode('WaypointModeAcc')

    time.sleep(10)
    shuttle.set_waypoint(initial_x + 3.0, initial_y, initial_z - 3.0, 0.0)

    time.sleep(10)
    shuttle.set_waypoint(initial_x + 3.0, initial_y + 3.0, initial_z - 3.0, 0.0)

    time.sleep(10)
    shuttle.set_waypoint(initial_x, initial_y + 3.0, initial_z - 3.0, 0.0)

    time.sleep(10)
    shuttle.set_waypoint(initial_x, initial_y, initial_z - 3.0, 0.0)

    time.sleep(10)
    '''
    # Land the drone
    real.set_autopilot_mode('OnboardLandMode')
    simul.set_autopilot_mode('OnboardLandMode')

    
    time.sleep(5)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
