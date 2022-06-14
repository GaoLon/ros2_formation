#include <iostream>
#include <math.h>
#include <string>
#include <random>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr _cmd_sub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _mesh_pub;
// ros::Subscriber _cmd_sub;
// ros::Publisher  _odom_pub;

quadrotor_msgs::msg::PositionCommand _cmd;
rclcpp::Parameter _init_x, _init_y, _init_z;
std::string mesh_resource; 

bool rcv_cmd = false;
// fstream file
void rcvPosCmdCallBack(const quadrotor_msgs::msg::PositionCommand cmd)
{	
	rcv_cmd = true;
	_cmd    = cmd;
}

void pubOdom(rclcpp::Node::SharedPtr nh)
{	
	nav_msgs::msg::Odometry odom;
	visualization_msgs::msg::Marker mesh;
	odom.header.stamp    = rclcpp::Time(nh->now(), RCL_SYSTEM_TIME);
	odom.header.frame_id = "world";
	mesh.header.stamp    = rclcpp::Time(nh->now(), RCL_SYSTEM_TIME);
	mesh.header.frame_id = "world";

	if(rcv_cmd)
	{
	    odom.pose.pose.position.x = _cmd.position.x;
	    odom.pose.pose.position.y = _cmd.position.y;
	    odom.pose.pose.position.z = _cmd.position.z;

		Eigen::Vector3d alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8*Eigen::Vector3d(0,0,1);
		Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
		Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
		Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
		Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
		Eigen::Vector3d zB = xB.cross(yB);
		Eigen::Matrix3d R;
		R.col(0) = xB;
		R.col(1) = yB;
		R.col(2) = zB;
		Eigen::Quaterniond q(R);
	    odom.pose.pose.orientation.w = q.w();
	    odom.pose.pose.orientation.x = q.x();
	    odom.pose.pose.orientation.y = q.y();
	    odom.pose.pose.orientation.z = q.z();

	    odom.twist.twist.linear.x = _cmd.velocity.x;
	    odom.twist.twist.linear.y = _cmd.velocity.y;
	    odom.twist.twist.linear.z = _cmd.velocity.z;

	    odom.twist.twist.angular.x = _cmd.acceleration.x;
	    odom.twist.twist.angular.y = _cmd.acceleration.y;
	    odom.twist.twist.angular.z = _cmd.acceleration.z;
	}
	else
	{
		odom.pose.pose.position.x = _init_x.as_double();
	    odom.pose.pose.position.y = _init_y.as_double();
	    odom.pose.pose.position.z = _init_z.as_double();

	    odom.pose.pose.orientation.w = 1;
	    odom.pose.pose.orientation.x = 0;
	    odom.pose.pose.orientation.y = 0;
	    odom.pose.pose.orientation.z = 0;

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;
	}

    _odom_pub->publish(odom);

	// mesh
	mesh.ns = "mesh";
	mesh.id = 0;
	mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	mesh.action = visualization_msgs::msg::Marker::ADD;
	mesh.pose = odom.pose.pose;
	mesh.scale.x = 1.0;
	mesh.scale.y = 1.0;
	mesh.scale.z = 1.0;
	mesh.color.a = 1;
	mesh.color.r = 0;
	mesh.color.g = 0;
	mesh.color.b = 0;
	mesh.mesh_resource = mesh_resource;

	_mesh_pub->publish(mesh);
}

int main (int argc, char** argv) 
{     
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("fake_drone", \
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

    nh->get_parameter_or("init_x", _init_x,  rclcpp::Parameter("init_x", 0.0));
    nh->get_parameter_or("init_y", _init_y,  rclcpp::Parameter("init_y", 0.0));
    nh->get_parameter_or("init_z", _init_z,  rclcpp::Parameter("init_z", 0.0));
    nh->get_parameter_or("mesh_resource", mesh_resource, std::string("package://fake_drone/meshes/hummingbird.mesh"));

    _cmd_sub  = nh->create_subscription<quadrotor_msgs::msg::PositionCommand>("position_cmd", 1, \
                        [](const quadrotor_msgs::msg::PositionCommand msg){rcvPosCmdCallBack(msg);});
    _odom_pub = nh->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);                      
    _mesh_pub = nh->create_publisher<visualization_msgs::msg::Marker>("mesh", 10);                
	
	rclcpp::Rate rate(100);
    // ros::Rate rate(100);
    bool status = rclcpp::ok();
    // bool status = ros::ok();
    while(status) 
    {
		pubOdom(nh);                   
        // ros::spinOnce();
		rclcpp::spin_some(nh);
        status = rclcpp::ok();
        rate.sleep();
    }

    return 0;
}