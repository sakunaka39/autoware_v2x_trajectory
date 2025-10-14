#ifndef V2X_APP_HPP_EUIC2VFR
#define V2X_APP_HPP_EUIC2VFR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <boost/asio/io_service.hpp>
// #include "autoware_v2x/cpm_application.hpp"
#include "autoware_v2x/time_trigger.hpp"
#include "autoware_v2x/link_layer.hpp"
#include "autoware_v2x/ethernet_device.hpp"
#include "autoware_v2x/positioning.hpp"
#include "autoware_v2x/security.hpp"
#include "autoware_v2x/router_context.hpp"
// #include "autoware_v2x/trajectory_application.hpp"
// #include "autoware_v2x/v2x_node.hpp"

namespace v2x_trajectory
{
  class V2XNode;
  class CpmApplication;

  class V2XApp
  {
  public:
    V2XApp(V2XNode *);
    void start();
    void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr);
    void tfCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr);

    CpmApplication *cp;
    // V2XNode *v2x_node;

  private:
    friend class Application;
    V2XNode* node_;
    bool tf_received_;
    double tf_interval_;
    bool app_started_;
  };
}

#endif