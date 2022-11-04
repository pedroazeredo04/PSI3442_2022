#include <ros/package.h>
#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>

#include "lib/drone.h"
#include "lib/vector_3d.h"

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "q1_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate{10};

    Drone drone{nh};

    int flight_route = 1;
    private_nh.getParam("flight_route", flight_route);

    // wait for FCU connection
    while(ros::ok() && !drone.get_mavros_state().connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool landed = false;
    bool offboard = false;
    while(ros::ok() and not landed) {
        if (drone.get_mavros_state().mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(drone.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                offboard = true;
            }
            last_request = ros::Time::now();

        } else {
            if (!drone.get_mavros_state().armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if (drone.arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed. Preparing to takeoff...");
                }
                last_request = ros::Time::now();
            }
        }
        
        static q1_fsm_state fsm_state = TAKEOFF;
        static bool first_state_time = true;

        if (drone.get_mavros_state().armed and offboard) {

            switch (fsm_state) {
                case TAKEOFF:
                    if (first_state_time) {
                        ROS_INFO("Takeoff started!");
                        drone.set_setpoint(Vector3D{0, 0, 1}.to_pose());
                        first_state_time = false;
                    }

                    if (Vector3D{0, 0, 1} == drone.get_pose()) {
                        ROS_INFO("Finished takeoff. Preparing to follow trajectory...");
                        fsm_state = FOLLOW_TRAJECTORY;
                        first_state_time = true;
                    }

                case FOLLOW_TRAJECTORY:

                    if (flight_route == 1) {
                        static bool first_track = true;

                        if (first_track) {
                            if (first_state_time) {
                                ROS_INFO("First track started!");
                                drone.set_setpoint(Vector3D{0, 5, 1}.to_pose());
                                first_state_time = false;
                            }

                            if (Vector3D{0, 5, 1} == drone.get_pose()) {
                                ROS_INFO("Finished first track. Starting second track...");
                                fsm_state = FOLLOW_TRAJECTORY;
                                first_track = false;
                                first_state_time = true;
                            }

                        } else {
                            if (first_state_time) {
                                ROS_INFO("Second track started!");
                                drone.set_setpoint(Vector3D{5, 5, 1}.to_pose());
                                first_state_time = false;
                            }

                            if (Vector3D{5, 5, 1} == drone.get_pose()) {
                                ROS_INFO("Finished second track. Landing...");
                                fsm_state = LANDING;
                                first_state_time = true;
                            }
                        }
                    }

                    if (flight_route == 2) {
                        if (first_state_time) {
                            ROS_INFO("Trajectory started!");
                            drone.set_setpoint(Vector3D{1, 1, 1}.to_pose());
                            first_state_time = false;
                        }

                        if (Vector3D{1, 1, 1} == drone.get_pose()) {
                            ROS_INFO("Finished trajectory. Landing...");
                            fsm_state = LANDING;
                            first_state_time = true;
                        }
                    }

                case LANDING:
                    if (first_state_time) {
                        ROS_INFO("Landing started!");
                        drone.set_setpoint(Vector3D{5, 5, 0}.to_pose());
                        first_state_time = false;
                    }

                    if (Vector3D{5, 5, 0} == drone.get_pose()) {
                        landed = true;
                        ROS_INFO("Landed. Successfully terminating controller.");
                    }
                }

            drone.pose_setpoint_pub.publish(drone.get_setpoint());

        } else {
            drone.pose_setpoint_pub.publish(Vector3D{0, 0, 0}.to_pose());
        }

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}