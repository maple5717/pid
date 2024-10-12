
#include <pid/pid.hpp>
#include "rclcpp/wait_for_message.hpp"
#include <chrono>

using namespace pid_ns;
using std::placeholders::_1;


PidObject::PidObject() 
: Node("pid_obj")
{
  this->declare_parameter<double>("Kp", 1.0);
  this->declare_parameter<double>("Ki", 0.0);
  this->declare_parameter<double>("Kd", 0.0);
  this->declare_parameter<double>("upper_limit", 1000.0);
  this->declare_parameter<double>("lower_limit", -1000.0);
  this->declare_parameter<double>("windup_limit", 1000.0);
  this->declare_parameter<double>("cutoff_frequency", -1.0);
  this->declare_parameter<std::string>("topic_from_controller", "control_effort");
  this->declare_parameter<std::string>("topic_from_plant", "state");
  this->declare_parameter<std::string>("setpoint_topic", "setpoint");
  this->declare_parameter<std::string>("pid_enable_topic", "pid_enable");
  this->declare_parameter<double>("max_loop_frequency", 1.0);
  this->declare_parameter<double>("min_loop_frequency", 1000.0);
  this->declare_parameter<std::string>("pid_debug_topic", "pid_debug");
  this->declare_parameter<double>("setpoint_timeout", -1.0);

  // Two parameters to allow for error calculation with discontinous value
  this->declare_parameter<bool>("angle_error", false);
  this->declare_parameter<double>("angle_wrap", 2.0 * 3.14159);

  // Get the parameters
  this->get_parameter("Kp", Kp_);
  this->get_parameter("Ki", Ki_);
  this->get_parameter("Kd", Kd_);
  this->get_parameter("upper_limit", upper_limit_);
  this->get_parameter("lower_limit", lower_limit_);
  this->get_parameter("windup_limit", windup_limit_);
  this->get_parameter("cutoff_frequency", cutoff_frequency_);
  this->get_parameter("topic_from_controller", topic_from_controller_);
  this->get_parameter("topic_from_plant", topic_from_plant_);
  this->get_parameter("setpoint_topic", setpoint_topic_);
  this->get_parameter("pid_enable_topic", pid_enable_topic_);
  this->get_parameter("max_loop_frequency", max_loop_frequency_);
  this->get_parameter("min_loop_frequency", min_loop_frequency_);
  this->get_parameter("pid_debug_topic", pid_debug_pub_name_);
  this->get_parameter("setpoint_timeout", setpoint_timeout_);
  this->get_parameter("angle_error", angle_error_);
  this->get_parameter("angle_wrap", angle_wrap_);

  // RCLCPP_ASSERT_MSG(setpoint_timeout_ == -1 || setpoint_timeout_ > 0,
  //                   "setpoint_timeout set to %.2f but needs to be -1 or >0", setpoint_timeout_);
  assert(setpoint_timeout_ == -1 || setpoint_timeout_ > 0);

rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
std::chrono::milliseconds(50), std::bind(&PidObject::doCalcs, this));


// Update params if specified as command-line options, & print settings
printParameters();
if (not validateParameters())
  std::cout << "Error: invalid parameter\n";

// instantiate publishers & subscribers
control_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(topic_from_controller_, 1); // node.advertise<std_msgs::Float64>(topic_from_controller_, 1);
pid_debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(pid_debug_pub_name_, 1);// node.advertise<std_msgs::Float64MultiArray>(pid_debug_pub_name_, 1);

plant_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    topic_from_plant_, 1, std::bind(&PidObject::plantStateCallback, this, _1));
setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    setpoint_topic_, 1, std::bind(&PidObject::setpointCallback, this, _1));
pid_enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    pid_enable_topic_, 1, std::bind(&PidObject::pidEnableCallback, this, _1));

if (!plant_sub_ || !setpoint_sub_ || !pid_enabled_sub_)
{
    RCLCPP_ERROR(this->get_logger(), "Initialization of a subscriber failed. Exiting.");
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
}

  // // dynamic reconfiguration
  // dynamic_reconfigure::Server<pid::PidConfig> config_server;
  // dynamic_reconfigure::Server<pid::PidConfig>::CallbackType f;
  // f = boost::bind(&PidObject::reconfigureCallback, this, _1, _2);
  // config_server.setCallback(f);rom




  // // Respond to inputs until shut down
  // while (rclcpp::ok())
  // {
  //   doCalcs();
  //   ros::spinOnce();

  //   // Add a small sleep to avoid 100% CPU usage
  //   ros::Duration(0.001).sleep();
  // }
};

void PidObject::waitFirstMsg()
{
  // Wait for first messages
  std_msgs::msg::Float64 out = std_msgs::msg::Float64();
  while (rclcpp::ok() && !rclcpp::wait_for_message(out, this->shared_from_this(), setpoint_topic_, std::chrono::seconds(10)))
  {
      RCLCPP_WARN(this->get_logger(), "Waiting for first setpoint message.");
  }

        // Wait for first state message from the plant
  while (rclcpp::ok() && !rclcpp::wait_for_message(out, this->shared_from_this(), topic_from_plant_, std::chrono::seconds(10)))
  {
      RCLCPP_WARN(this->get_logger(), "Waiting for first state message from the plant.");
  }
}

void PidObject::setpointCallback(const std_msgs::msg::Float64::SharedPtr setpoint_msg)
{
  setpoint_ = setpoint_msg->data;
  last_setpoint_msg_time_ = this->now().seconds();
  new_state_or_setpt_ = true;
}

void PidObject::plantStateCallback(const std_msgs::msg::Float64::SharedPtr state_msg)
{
  plant_state_ = state_msg->data;

  new_state_or_setpt_ = true;

  doCalcs();
}

void PidObject::pidEnableCallback(const std_msgs::msg::Bool::SharedPtr pid_enable_msg)
{
  pid_enabled_ = pid_enable_msg->data;
}

void PidObject::getParams(double in, double& value, double& scale)
{
  int digits = 0;
  value = in;
  while (rclcpp::ok() && ((fabs(value) > 1.0 || fabs(value) < 0.1) && (digits < 2 && digits > -1)))
  {
    if (fabs(value) > 1.0)
    {
      value /= 10.0;
      digits++;
    }
    else
    {
      value *= 10.0;
      digits--;
    }
  }
  if (value > 1.0)
    value = 1.0;
  if (value < -1.0)
    value = -1.0;

  scale = pow(10.0, digits);
}

bool PidObject::validateParameters()
{
  if (lower_limit_ > upper_limit_)
  {
    RCLCPP_ERROR(this->get_logger(), "The lower saturation limit cannot be greater than the upper "
              "saturation limit.");
    return (false);
  }

  return true;
}

void PidObject::printParameters()
{
  std::cout << std::endl << "PID PARAMETERS" << std::endl << "-----------------------------------------" << std::endl;
  std::cout << "Kp: " << Kp_ << ",  Ki: " << Ki_ << ",  Kd: " << Kd_ << std::endl;
  if (cutoff_frequency_ == -1)  // If the cutoff frequency was not specified by the user
    std::cout << "LPF cutoff frequency: 1/4 of sampling rate" << std::endl;
  else
    std::cout << "LPF cutoff frequency: " << cutoff_frequency_ << std::endl;
  std::cout << "pid node name: " << this->get_name() << std::endl;
  std::cout << "Name of topic from controller: " << topic_from_controller_ << std::endl;
  std::cout << "Name of topic from the plant: " << topic_from_plant_ << std::endl;
  std::cout << "Name of setpoint topic: " << setpoint_topic_ << std::endl;
  std::cout << "Integral-windup limit: " << windup_limit_ << std::endl;
  std::cout << "Saturation limits: " << upper_limit_ << "/" << lower_limit_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl;

  return;
}

// void PidObject::reconfigureCallback(pid::PidConfig& config, uint32_t level)
// {
//   if (first_reconfig_)
//   {
//     getParams(Kp_, config.Kp, config.Kp_scale);
//     getParams(Ki_, config.Ki, config.Ki_scale);
//     getParams(Kd_, config.Kd, config.Kd_scale);
//     first_reconfig_ = false;
//     return;  // Ignore the first call to reconfigure which happens at startup
//   }

//   Kp_ = config.Kp * config.Kp_scale;
//   Ki_ = config.Ki * config.Ki_scale;
//   Kd_ = config.Kd * config.Kd_scale;
//   ROS_INFO("Pid reconfigure request: Kp: %f, Ki: %f, Kd: %f", Kp_, Ki_, Kd_);
// }

void PidObject::doCalcs()
{
  // std::cout << "lolol" << 123 << std::endl; // for test only!!!
  if (!pid_enabled_)
    {
      // TODO: set integral to 0 once disabled 
      error_integral_ = 0;
      control_msg_.data = 0;
      control_effort_pub_->publish(control_msg_);
      return;
      }

  
  // Do fresh calcs if knowledge of the system has changed.
  if (new_state_or_setpt_)
  {
    if (!((Kp_ <= 0. && Ki_ <= 0. && Kd_ <= 0.) ||
          (Kp_ >= 0. && Ki_ >= 0. && Kd_ >= 0.)))  // All 3 gains should have the same sign
      RCLCPP_WARN(this->get_logger(), "All three gains (Kp, Ki, Kd) should have the same sign for "
               "stability.");

    error_.at(2) = error_.at(1);
    error_.at(1) = error_.at(0);
    error_.at(0) = setpoint_ - plant_state_;  // Current error goes to slot 0

    // If the angle_error param is true, then address discontinuity in error
    // calc.
    // For example, this maintains an angular error between -180:180.
    if (angle_error_)
    {
      while (error_.at(0) < -1.0 * angle_wrap_ / 2.0)
      {
        error_.at(0) += angle_wrap_;

        // The proportional error will flip sign, but the integral error
        // won't and the filtered derivative will be poorly defined. So,
        // reset them.
        error_deriv_.at(2) = 0.;
        error_deriv_.at(1) = 0.;
        error_deriv_.at(0) = 0.;
        error_integral_ = 0.;
      }
      while (error_.at(0) > angle_wrap_ / 2.0)
      {
        error_.at(0) -= angle_wrap_;

        // The proportional error will flip sign, but the integral error
        // won't and the filtered derivative will be poorly defined. So,
        // reset them.
        error_deriv_.at(2) = 0.;
        error_deriv_.at(1) = 0.;
        error_deriv_.at(0) = 0.;
        error_integral_ = 0.;
      }
    }

    // calculate delta_t
    if (!prev_time_ == 0)  // Not first time through the program
    {
      delta_t_ = this->now().seconds() - prev_time_;
      prev_time_ = this->now().seconds();
      if (0 == delta_t_)
      {
        RCLCPP_ERROR(this->get_logger(), "delta_t is 0, skipping this loop. Possible overloaded cpu "
                  "at time: %f",
                  this->now().seconds());
        return;
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "prev_time is 0, doing nothing");
      prev_time_ = this->now().seconds();
      return;
    }

    // integrate the error
    error_integral_ += error_.at(0) * delta_t_;

    // Apply windup limit to limit the size of the integral term
    if (error_integral_ > fabsf(windup_limit_))
      error_integral_ = fabsf(windup_limit_);

    if (error_integral_ < -fabsf(windup_limit_))
      error_integral_ = -fabsf(windup_limit_);

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
    if (cutoff_frequency_ != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt_ = tan((cutoff_frequency_ * 6.2832) * delta_t_ / 2);

      // Avoid tan(0) ==> NaN
      if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
        tan_filt_ = -0.01;
      if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
        tan_filt_ = 0.01;

      c_ = 1 / tan_filt_;
    }

    filtered_error_.at(2) = filtered_error_.at(1);
    filtered_error_.at(1) = filtered_error_.at(0);
    filtered_error_.at(0) = error_.at(0); //(1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.at(2) + 2 * error_.at(1) + error_.at(0) -
                                                                // (c_ * c_ - 1.414 * c_ + 1) * filtered_error_.at(2) -
                                                                // (-2 * c_ * c_ + 2) * filtered_error_.at(1));

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_.at(2) = error_deriv_.at(1);
    error_deriv_.at(1) = error_deriv_.at(0);
    error_deriv_.at(0) = (error_.at(0) - error_.at(1)) / delta_t_;

    filtered_error_deriv_.at(2) = filtered_error_deriv_.at(1);
    filtered_error_deriv_.at(1) = filtered_error_deriv_.at(0);

    filtered_error_deriv_.at(0) =
        (1 / (1 + c_ * c_ + 1.414 * c_)) *
        (error_deriv_.at(2) + 2 * error_deriv_.at(1) + error_deriv_.at(0) -
         (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_.at(2) - (-2 * c_ * c_ + 2) * filtered_error_deriv_.at(1));

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_.at(0);
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_.at(0);
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > upper_limit_)
      control_effort_ = upper_limit_;
    else if (control_effort_ < lower_limit_)
      control_effort_ = lower_limit_;

    // Publish the stabilizing control effort if the controller is enabled
    if (pid_enabled_ && (setpoint_timeout_ == -1 || 
                         (this->now().seconds() - last_setpoint_msg_time_) <= setpoint_timeout_))
    {
      control_msg_.data = control_effort_;
      control_effort_pub_->publish(control_msg_);
      // Publish topic with
      std::vector<double> pid_debug_vect { plant_state_, control_effort_, proportional_, integral_, derivative_};
      std_msgs::msg::Float64MultiArray pidDebugMsg;
      pidDebugMsg.data = pid_debug_vect;
      pid_debug_pub_->publish(pidDebugMsg);
    }
    else if (setpoint_timeout_ > 0 && (this->now().seconds() - last_setpoint_msg_time_) > setpoint_timeout_)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Setpoint message timed out, will stop publising control_effort_messages");
      error_integral_ = 0.0;
    } 
    else
      error_integral_ = 0.0;
  }

  new_state_or_setpt_ = false;
}
