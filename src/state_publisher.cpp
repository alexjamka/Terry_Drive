#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono;

class StatePublisher : public rclcpp::Node{
    public:

    StatePublisher(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        Node("state_publisher",options){
            joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
            broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            RCLCPP_INFO(this->get_logger(),"Starting Buddy state publisher");

            timer_=this->create_wall_timer(33ms,std::bind(&StatePublisher::publish,this));
        }

        void publish();
    private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    rclcpp::TimerBase::SharedPtr timer_;

    const double degree=M_PI/180.0;
    double wheel_angle = 0.;
    double robot_angle = 0.;
};

void StatePublisher::publish(){
    geometry_msgs::msg::TransformStamped t;
    sensor_msgs::msg::JointState joint_state;

    // Publish wheel joint states
    joint_state.header.stamp=this->get_clock()->now();
    joint_state.name={"wheel_to_base_right","wheel_to_base_left"};
    joint_state.position={wheel_angle, wheel_angle};

    // Broadcast odom -> base transform (robot driving in a circle)
    t.header.stamp=this->get_clock()->now();
    t.header.frame_id="odom";
    t.child_frame_id="BuddyBase_link";

    t.transform.translation.x=cos(robot_angle)*1.0;
    t.transform.translation.y=sin(robot_angle)*1.0;
    t.transform.translation.z=0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,robot_angle+M_PI/2);
    t.transform.rotation.x=q.x();
    t.transform.rotation.y=q.y();
    t.transform.rotation.z=q.z();
    t.transform.rotation.w=q.w();

    // Update for next tick
    wheel_angle+=5.0*degree;   // wheels spin
    robot_angle+=0.5*degree;   // robot drives in circle

    broadcaster->sendTransform(t);
    joint_pub_->publish(joint_state);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}