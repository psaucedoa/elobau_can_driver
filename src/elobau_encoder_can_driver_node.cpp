#include <string>
#include "elobau_encoder_can_driver/elobau_encoder_can_driver_node.hpp"

namespace rlc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace drivers
{
namespace elobau
{

ElobauEncoderCanDriverNode::ElobauEncoderCanDriverNode(const rclcpp::NodeOptions & OPTIONS)
: generic_can_driver::GenericCanDriverNode(OPTIONS)
{
  // params
  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  device_ID_ = this->declare_parameter<uint8_t>("device_ID", 0);
  sub_topic_can_ = this->declare_parameter<std::string>("can_sub_topic", "");
  pub_topic_can_ = this->declare_parameter<std::string>("pub_topic_can", "");

  dbw_dbc_db = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  RCLCPP_INFO(this->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "device_id: %d", device_ID_);
  RCLCPP_INFO(this->get_logger(), "sub_topic_can: %s", sub_topic_can_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_can: %s", pub_topic_can_.c_str());
}

ElobauEncoderCanDriverNode::~ElobauEncoderCanDriverNode()
{}

LNI::CallbackReturn ElobauEncoderCanDriverNode::on_configure(const rlc::State & state)
{
  LNI::on_configure(state);
  try
  {
    // setup subscribers
    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        sub_topic_can_, 500, std::bind(&ElobauEncoderCanDriverNode::rxFrame, this,
        std::placeholders::_1));

    // setup publishers
    pub_topic_joint_ = pub_topic_joint_ + sensor_name_;
    pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>(pub_topic_joint_, 20);
    pub_can_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_can_, 20);
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error w/ on_configure: %s", e.what());
    return LNI::CallbackReturn::FAILURE;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Successfully Configured!");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn ElobauEncoderCanDriverNode::on_activate(const rlc::State & state)
{
  LNI::on_activate(state);
  // when driver activates, configrue the device
  pub_joint_->on_activate();
  pub_can_->on_activate();

  RCLCPP_DEBUG(this->get_logger(), "Elobau Encoder activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn ElobauEncoderCanDriverNode::on_deactivate(const rlc::State & state)
{
  // (void)state;

  LNI::on_deactivate(state);

  pub_joint_->on_deactivate();

  RCLCPP_DEBUG(this->get_logger(), "Elobau Encoder deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn ElobauEncoderCanDriverNode::on_cleanup(const rlc::State & state)
{
  // (void)state;

  LNI::on_cleanup(state);

  RCLCPP_DEBUG(this->get_logger(), "Elobau Encoder cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn ElobauEncoderCanDriverNode::on_shutdown(const rlc::State & state)
{
  // (void)state;

  LNI::on_shutdown(state);

  RCLCPP_DEBUG(this->get_logger(), "Elobau Encoder shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void ElobauEncoderCanDriverNode::rxFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  if(!MSG->is_rtr && !MSG->is_error && (device_ID_ == (MSG->id & 0x000000FFu)))
  {
    // RCLCPP_INFO(this->get_logger(), "First Byte: %d, Second Byte: %d", MSG->data[0], MSG->data[1]);
    switch (MSG->id & 0xFFFFFF00u)
    {
      case ID_elobau_encoder:
        recvEncoderFrame(MSG);
        break;
      default:
        break;
    }
  }
}

void ElobauEncoderCanDriverNode::recvEncoderFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("EncoderPosition");

  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc())
  {
    message->SetFrame(MSG);

    // populating header
    joint_state_out_.header.stamp = MSG->header.stamp;
    joint_state_out_.header.frame_id = frame_id_;
    joint_state_out_.name = {sensor_name_};

    uint16_t position_pos;
    position_pos = (MSG->data[0] << 8) | MSG->data[1];
    joint_state_out_.position = {static_cast<double>(position_pos)/10.};

    // uint16_t position_neg;
    // position_neg = (MSG->data[2] << 8) | MSG->data[3];
    // joint_state_out_.velocity = {static_cast<double>(position_neg)/10.};

    pub_joint_->publish(joint_state_out_);
  }
}


}  // end namespace elobau
}  // end namespace drivers

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::elobau::ElobauEncoderCanDriverNode)