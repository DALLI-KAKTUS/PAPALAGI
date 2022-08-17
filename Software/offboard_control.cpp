#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>



#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <math.h>

float X;
float Y;	

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
     
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		
	

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		offboard_setpoint_counter_ = 0;

	

		auto sendCommands = [this]() -> void {
			if (waypointIndex == 0 && temp != 2){
				for(int i=0;i<2000000;i++){
					std::cout << "waiting: " << i << std::endl;
				};
				waypointIndex++;
			};
			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				//this->arm();	

			};
//-------------
			subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"/fmu/vehicle_local_position/out",
#ifdef ROS_DEFAULT_API
            10,
#endif
			[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			X = msg->x;
			Y = msg->y;

			std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE GPS POSITION DATA"   << std::endl;
			std::cout << "=================================="   << std::endl;
			std::cout << "ts: "      << msg->timestamp    << std::endl;
			//std::cout << "lat: " << msg->x  << std::endl;
			//std::cout << "lon: " << msg->y << std::endl;
			std::cout << "lat: " << X  << std::endl;
			std::cout << "lon: " << Y << std::endl;
			std::cout << "waypoint: " << vec[waypointIndex][0] << std::endl;
			std::cout << "waypoint: " << vec[waypointIndex][1] << std::endl;
			std::cout << "index: " << waypointIndex << std::endl;

			
			/*
			if((waypoints[waypointIndex][0] + 0.3 > X && waypoints[waypointIndex][0] - 0.3 < X)&&(waypoints[waypointIndex][1] + 0.3 > Y && waypoints[waypointIndex][1] - 0.3 < Y)){
			waypointIndex++;
			
			if (waypointIndex >= waypoints.size())
				exit(0);
				//waypointIndex = 0;
			
			if (waypointIndex == 2){
				std::system("gnome-terminal -- bash -c 'source ~/px4_ros_com_ros2/install/setup.bash && ros2 launch px4_ros_com sensor_combined_listener.launch.py'");
			}
			
			RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f", waypoints[waypointIndex][0], waypoints[waypointIndex][1], waypoints[waypointIndex][2]);
			}
			*/
			if (temp == 0){
			centerX = X;
			centerY = Y;
			while (theta <= thetaMax)
			{

			double away = awayStep * theta;

			double around = theta + rotation;	// how far around the center

			float x = centerX + cos (around) * away;
			float y = centerY + sin (around) * away;

			theta += chord / away;
			vec[i][0] = x;
			vec[i][1] = y;
			vec[i][2] = -10;
			i++;
			}
			for(int i=0;i<500000;i++){
				std::cout << vec[0][0] << std::endl;
				std::cout << vec[0][1] << std::endl;
				};
			temp++;
			};
			if (temp == 2){
				if(waypointIndex == 30){
					RCLCPP_INFO(this->get_logger(), "Birakildi");
				};
				if(waypointIndex>=45){
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
					exit(0);

				};

			};

			
		});
//--------------
			
            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

           		 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};

		
		auto nextWaypoint = [this]() -> void {
			
			waypointIndex++; 
			
			/*
			//EA
			if (waypointIndex >= 200){
				//this->publish_vehicle_command(VehicleCommand::MAV_CMD_DO_VTOL_TRANSITION, 1, 6);
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
			};
			*/
			if (waypointIndex >= 232) {
				waypointIndex = 0;
				temp++;
				for(int i=0;i<232;i++){
					vec[i][0] = centerX+2;
					vec[i][1] = centerY+2;
					vec[i][2] = -5;
				};
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
				//exit(0);
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f", vec[waypointIndex][0], vec[waypointIndex][1], vec[waypointIndex][2]);
			};
		};
		
		commandTimer = this->create_wall_timer(100ms, sendCommands);
		waypointTimer = this->create_wall_timer(1s, nextWaypoint); //EA	
	}

	void arm() const;
	void disarm() const;
	void topic_callback() const;
private:


	int waypointIndex = 0;

/////////////////////////////////////////////////////
	
	int radius = 10;

	int coils = 3;		// how many turns

	float centerX = 0;
	float centerY = 0;

	float thetaMax = coils * 2 * M_PI;
	float awayStep = radius / thetaMax;
	float chord = 0.4;
	float theta = chord / awayStep;

	int rotation = 0;
	int i = 0;
	
 ///////////////////////////////////////////////////
	
	
	int temp = 0;

	float vec [232][3];
	rclcpp::TimerBase::SharedPtr commandTimer;
	rclcpp::TimerBase::SharedPtr waypointTimer;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	//
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	//
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
};

void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}	

void OffboardControl::publish_trajectory_setpoint() const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.position = {vec[waypointIndex][0],vec[waypointIndex][1],vec[waypointIndex][2]};
	msg.yaw = std::nanf("0");		//-3.14; // [-PI:PI]
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting setpoint node..." << std::endl;
	
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}

//238