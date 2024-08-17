import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pegasus_msgs.srv import Waypoint, AddCircle, SetMode
from pegasus_msgs.msg import AutopilotStatus
from std_msgs.msg import String


class Drone(Node):

    def __init__(self, id):

        super().__init__('drone_api_' + str('id'))

        self.id = id
        self.namespace = 'drone'

        # Create the service clients for the drone
        self.add_waypoint_srv = self.create_client(Waypoint, '/drone' + str(id) +'/autopilot/set_waypoint')
        #print('Initializing service: /drone' + str(id) +'/autopilot/set_waypoint')
        while not self.add_waypoint_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not availtimeout_secable, waiting again...')

        self.add_circle_srv = self.create_client(AddCircle, '/drone' + str(id) +'/autopilot/trajectory/add_circle')
        #print('Initializing service: /drone' + str(id) +'/autopilot/add_circle')
        while not self.add_circle_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')

        self.set_autopilot_srv = self.create_client(SetMode, '/drone' + str(id) +'/autopilot/change_mode')
        #print('Initializing service: /drone' + str(id) +'/autopilot/change_mode')
        while not self.set_autopilot_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('service not available, waiting again...')
        '''
        self.get_autopilot_status_srv = self.create_client(AutopilotStatus, '/drone' + str(id) +'/autopilot/get_status')
        while not self.get_autopilot_status_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        '''
        # Create subscriptions
        self.create_subscription(AutopilotStatus, '/drone' + str(id) + '/autopilot/status', self.autopilot_status_callback, qos_profile_sensor_data)

        # Requests messages
        self.waypoint_req = Waypoint.Request()
        self.circle_req = AddCircle.Request()
        self.set_mode_req = SetMode.Request()

        # Instance variable to store autopilot status
        #self.autopilot_status = None

    def autopilot_status_callback(self, msg):
        self.get_logger().info('Received autopilot status: %s' % msg.mode)
        # Update the instance variable
        #self.autopilot_status = msg.mode

    def set_autopilot_mode(self, mode='DisarmMode'):

        self.get_logger().info('Setting autopilot mode to: %s' % mode)
            
        # Set the mode request
        self.set_mode_req.mode = mode
        
        # Make an async request
        self.future = self.set_autopilot_srv.call_async(self.set_mode_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def set_waypoint(self, x, y, z, yaw):

        self.get_logger().info('Setting waypoint to: %f, %f, %f, %f' % (x, y, z, yaw))
        
        # Set the waypoint request
        self.waypoint_req.position[0] = x
        self.waypoint_req.position[1] = y
        self.waypoint_req.position[2] = z
        self.waypoint_req.yaw = yaw
        
        # Make an async request
        self.future = self.add_waypoint_srv.call_async(self.waypoint_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):

    # Initiate the ROS2 nodes
    rclpy.init(args=args)

    # Create the drone
    shuttle = Drone(1)
    target = Drone(2)

    shuttle.set_autopilot_mode('ArmMode')
    shuttle.set_autopilot_mode('TakeoffMode')

    target.set_autopilot_mode('ArmMode')
    target.set_autopilot_mode('TakeoffMode')

    time.sleep(5)

    shuttle.set_autopilot_mode('CaptureTargetMode')

    target.set_waypoint(  0.0, 0.0, -10, 0.0)
    target.set_autopilot_mode('WaypointMode')

    time.sleep(10)
    target.set_waypoint(  30.0, 0.0, -10, 0.0)
    target.set_autopilot_mode('WaypointMode')
    
    time.sleep(10)

    target.set_waypoint(  30.0, 5.0, -10, 0.0)
    target.set_autopilot_mode('WaypointMode')
    
    time.sleep(10)

    target.set_waypoint(  0.0, 5.0, -10, 0.0)
    target.set_autopilot_mode('WaypointMode')
    
    time.sleep(10)

    target.set_waypoint(  0.0, 5.0, -1, 0.0)
    target.set_autopilot_mode('WaypointMode')
    
    
    time.sleep(10)

    target.set_autopilot_mode('LandMode')

    rclpy.shutdown()


if __name__ == "__main__":
    main()