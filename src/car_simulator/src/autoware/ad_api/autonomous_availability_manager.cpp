#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tier4_system_msgs/msg/mode_change_available.hpp"

class AutonomousAvailabilityManager : public rclcpp::Node
{
private:
	static inline const std::string module_names[] = {
	"sensing", "perception", "map", "localization", "planning", "control", "vehicle", "system",
	};

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<tier4_system_msgs::msg::ModeChangeAvailable>::SharedPtr publishers_[std::size(module_names)];

	int publish_period_;
public:
	AutonomousAvailabilityManager()
	: Node("autonomous_availability_manager")
	{
		publish_period_ = this->declare_parameter<int>("publish_period", 100);

		RCLCPP_INFO_THROTTLE(
			get_logger(),
			*get_clock(),
			std::chrono::milliseconds(1000).count(),
			"Publishing autonomous availibitily all %d miliseconds",
			publish_period_
		);
		
		timer_ = rclcpp::create_timer(
			this,
			this->get_clock(),
			std::chrono::milliseconds(this->publish_period_),
			std::bind(&AutonomousAvailabilityManager::timeCallback, this)
		);
		
		for(size_t i = 0; i < std::size(module_names); ++i){
			const auto name = "/system/component_state_monitor/component/autonomous/" + module_names[i];
			
			publishers_[i] = create_publisher<tier4_system_msgs::msg::ModeChangeAvailable>(
				name,
				rclcpp::QoS(1).transient_local()
			);
		}
	}

private:
	void timeCallback()
	{
		tier4_system_msgs::msg::ModeChangeAvailable state;
		state.stamp = this->now();
		state.available = true;
		
		for(size_t i = 0; i < std::size(publishers_); ++i){
			publishers_[i]->publish(state);
		}
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AutonomousAvailabilityManager>());
	rclcpp::shutdown();
	return 0;
}