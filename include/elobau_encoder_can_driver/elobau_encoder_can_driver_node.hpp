#ifndef ELOBAU_ENCODER_CAN_DRIVER__ELOBAU_ENCODER_CAN_DRIVER_NODE_HPP_
#define ELOBAU_ENCODER_CAN_DRIVER__ELOBAU_ENCODER_CAN_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "generic_can_driver/generic_can_driver_node.hpp"
// #include "elobau_encoder_can_driver/visibility_control.hpp"

using namespace std::chrono_literals;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace drivers
{

namespace elobau
{

class ElobauEncoderCanDriverNode : public generic_can_driver::GenericCanDriverNode
{
public:
  explicit ElobauEncoderCanDriverNode(const rclcpp::NodeOptions & OPTIONS);

  ~ElobauEncoderCanDriverNode();

  // These are the can IDs for acceleration, angular rate, and slope can frames, minus device ID
  enum PROPRIETARY_CANIDs
  {
    // make sure to make the last byte 00 ALWAYS
    ID_elobau_encoder = 0x18FF0B00u,
  };

  /**
   * @brief Configures the driver. Sets up params, creates publishers.
  */
  LNI::CallbackReturn on_configure(const rlc::State & state);

  /**
   * @brief Activates the driver. Activates publishers, configures device.
  */
  LNI::CallbackReturn on_activate(const rlc::State & state);

  /**
   * @brief Deactivates the driver. Deactivates publisher.
  */
  LNI::CallbackReturn on_deactivate(const rlc::State & state);

  /**
   * @brief Performs Cleanup on the driver node. Resets to "as-new" state
  */
  LNI::CallbackReturn on_cleanup(const rlc::State & state);

  /**
   * @brief Shutsdown the driver.
  */
  LNI::CallbackReturn on_shutdown(const rlc::State & state);

private:
  /**
   * @brief Receives all incoming can Frames with matching device_ID_
   * @param MSG Standard ROS2 can frame message. J1939
  */
  void rxFrame(const can_msgs::msg::Frame::SharedPtr MSG);
  void recvEncoderFrame(const can_msgs::msg::Frame::SharedPtr MSG);

  sensor_msgs::msg::JointState joint_state_out_;

  // publishers
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::JointState>> pub_joint_;

  // params
  std::string pub_topic_joint_ = "/joint/elobau/rotary_encoder/";
};  // end class ElobauEncoderCanDriverNode
}  // end namespace elobau
}  // end namespace drivers
#endif