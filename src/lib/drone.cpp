#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "lib/drone.h"

Drone::Drone(ros::NodeHandle nh) {
    static auto set_state_callback = [this](const mavros_msgs::State::ConstPtr& msg) {
        this->set_state(msg);
    };

    static auto set_pose_callback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        this->set_pose(msg);
    };

    // Subscribers 
    this->state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, set_state_callback);
    this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, set_pose_callback);

    // Publishers
    this->pose_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // Services
    this->arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    this->set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
}

void Drone::set_setpoint(geometry_msgs::PoseStamped setpoint) {
    this->setpoint = setpoint;
}

geometry_msgs::PoseStamped Drone::get_setpoint() {
    return this->setpoint;
}

void Drone::set_state(const mavros_msgs::State::ConstPtr& msg) {
    this->mavros_state = *msg;
}

void Drone::set_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    this->pose = *msg;
}

geometry_msgs::PoseStamped Drone::get_pose() {
    return this->pose;
}

mavros_msgs::State Drone::get_mavros_state() {
    return this->mavros_state;
}
