
#pragma once
#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_service_client.hpp>
#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <ros_timer_client/cl_ros_timer.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cr_interfaces/srv/get_next_waypoint.hpp>

namespace sm_trial_planner
{
using namespace smacc2::client_bases;
class OrSubscribeMeasurement : public smacc2::Orthogonal<OrSubscribeMeasurement>
{
public:
    void onInitialize() override
    {
        auto subscribe_client = this->createClient<SmaccSubscriberClient<std_msgs::msg::Float32>>("moisture_percent");
    }
};
} // namespace sm_trial_planner
