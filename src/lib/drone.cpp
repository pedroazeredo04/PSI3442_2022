#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "lib/drone.h"
#include "lib/vector_3d.h"

Drone::Drone(ros::NodeHandle nh) :
    energy{0},
    setpoint{Vector3D{0}.to_pose()},
    pose{Vector3D{0}.to_pose()} {
    static auto set_state_callback = [this](const mavros_msgs::State::ConstPtr& msg) {
                                         this->set_state(msg);
                                     };

    static auto set_pose_callback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                                        this->set_pose(msg);
                                    };

    static auto set_battery_callback = [this](const sensor_msgs::BatteryState::ConstPtr& msg) {
                                           this->set_battery(msg);
                                       };

    // Subscribers
    this->state_sub = nh.subscribe<mavros_msgs::State>
                          ("mavros/state", 10, set_state_callback);
    this->pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                         ("mavros/local_position/pose", 10, set_pose_callback);
    this->battery_sub = nh.subscribe<sensor_msgs::BatteryState>
                            ("mavros/battery", 10, set_battery_callback);

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

void Drone::set_battery(const sensor_msgs::BatteryState::ConstPtr& msg) {
    this->battery = *msg;
}

void Drone::calculate_energy() {
    this->energy += this->battery.current * this->battery.voltage / 10;
}

double Drone::get_energy() {
    return this->energy;
}

geometry_msgs::PoseStamped Drone::get_pose() {
    return this->pose;
}

mavros_msgs::State Drone::get_mavros_state() {
    return this->mavros_state;
}
