#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tier4_control_msgs/msg/external_command_selector_mode.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

class ExternalCmdSelectorConversion : public rclcpp::Node
{
private:
	rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr operation_mode_sub_;
	rclcpp::Publisher<tier4_control_msgs::msg::ExternalCommandSelectorMode>::SharedPtr external_cmd_selector_mode_pub_;

	int publish_period_;
public:
	ExternalCmdSelectorConversion()
	: Node("external_cmd_selector_conversion")
	{
		using std::placeholders::_1;
		
		operation_mode_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
			"~/input/operation_mode",
			rclcpp::QoS{1},
			std::bind(&ExternalCmdSelectorConversion::callbackOperationMode, this, _1)
		);
		
		external_cmd_selector_mode_pub_ = create_publisher<tier4_control_msgs::msg::ExternalCommandSelectorMode>(
			"~/output/selector_mode",
			rclcpp::QoS{1}
		);
	}

private:
	void callbackOperationMode(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode)
	{
		tier4_control_msgs::msg::ExternalCommandSelectorMode mode;
		switch(operation_mode->mode){
			case autoware_adapi_v1_msgs::msg::OperationModeState::UNKNOWN:
				mode.data = tier4_control_msgs::msg::ExternalCommandSelectorMode::NONE;
				break;
			case autoware_adapi_v1_msgs::msg::OperationModeState::STOP:
				mode.data = tier4_control_msgs::msg::ExternalCommandSelectorMode::NONE;
				break;
			case autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS:
				mode.data = tier4_control_msgs::msg::ExternalCommandSelectorMode::NONE;
				break;
			case autoware_adapi_v1_msgs::msg::OperationModeState::LOCAL:
				mode.data = tier4_control_msgs::msg::ExternalCommandSelectorMode::LOCAL;
				break;
			case autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE:
				mode.data = tier4_control_msgs::msg::ExternalCommandSelectorMode::REMOTE;
				break;
			default:
				throw std::runtime_error("Reached unreachable state");
		}
		external_cmd_selector_mode_pub_->publish(mode);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ExternalCmdSelectorConversion>());
	rclcpp::shutdown();
	return 0;
}