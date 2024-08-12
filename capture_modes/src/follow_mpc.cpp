#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <math.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <casadi/casadi.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "/home/ricardo/ws_offboard_control/src/px4_ros_com/include/px4_ros_com/gen.h"
#include "px4_ros_com/gen.h"



using namespace casadi;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
class setpoint : public rclcpp::Node {
public:


    setpoint() : Node("setpoint") {
    

    	     
	    auto sendCommands = [this]() -> void {
		    
			//shuttle
		    P[0]=X2;
		    P[1]=Y2;
		    P[2]=Z2;
		    V[0]=vx2;
		    V[1]=vy2;
		    V[2]=vz2;

			//target
			Pd[0]=X3;
		    Pd[1]=Y3;
		    Pd[2]=Z3;
		    Vd[0]=vx3;
		    Vd[1]=vy3;
		    Vd[2]=vz3;


                   // offboard_control_mode needs to be paired with trajectory_setpoint
		    publish_offboard_control_mode();
		    publish_trajectory_setpoint();

                   // stop the counter after reaching 11
		    if (offboard_setpoint_counter_ < 11) {
			    offboard_setpoint_counter_++;
		    }
	    };

	
	    auto nextWaypoint = [this]() -> void {
		    
			if(mode==1){
				

				xx0 = {X2, Y2, Z2, vx2, vy2, vz2, dir2, X3, Y3, Z3, vx3, vy3, vz3, dir3};

                //auto start_time = rclcpp::Clock().now();

			
	    };
	
	    commandTimer = this->create_wall_timer(100ms, sendCommands);
	    waypointTimer = this->create_wall_timer(100ms, nextWaypoint); //EA	
	    
    }
private:
   

    rclcpp::TimerBase::SharedPtr commandTimer;
    rclcpp::TimerBase::SharedPtr waypointTimer;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    //
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription2_;
    //
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode() const;
    void publish_trajectory_setpoint() const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
};