#include <ros/package.h>
#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>

#include "lib/drone.h"
#include "lib/vector_3d.h"

static constexpr double altitude{5.0};
static ros::Duration tolerance_time{5.0};

static std::vector<Vector3D> desired_points;

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "q1_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate{10};

    Drone drone{nh};

    int flight_route = 1;
    private_nh.getParam("flight_route", flight_route);

    desired_points = std::vector<Vector3D>{
        Vector3D{0, 0, altitude},
        Vector3D{4, 4, altitude},
        Vector3D{0, 0, altitude}
    };

    // wait for FCU connection
    while (ros::ok() && !drone.get_mavros_state().connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Declares mavros controllers
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    static ros::Time last_request = ros::Time::now();
    static ros::Time state_enter_time = ros::Time::now();

    static bool landed = false;
    static bool offboard = false;
    static bool landing = false;
    static bool first_time_state = true;
    static int mission_number = 0;

    while (ros::ok() and not landed) {
        if ((drone.get_mavros_state().mode != "OFFBOARD") and
            (ros::Time::now() - last_request > ros::Duration(5.0)) and
            not offboard) {
            if (drone.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
                offboard = true;
            }

            last_request = ros::Time::now();
        } else {
            if (!drone.get_mavros_state().armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (drone.arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed. Preparing to takeoff...");
                }

                last_request = ros::Time::now();
            }
        }

        if (drone.get_mavros_state().armed and offboard and not landing) {
            if (first_time_state) {
                ROS_INFO("New setpoint sent");
                drone.set_setpoint(desired_points[mission_number].to_pose());
                first_time_state = false;
                state_enter_time = ros::Time::now();
            }

            if (desired_points[mission_number] == drone.get_pose() and
                ros::Time::now() - state_enter_time > tolerance_time) {
                mission_number += 1;
                first_time_state = true;

                // std::cout << "Current drone energy: " << drone.get_energy() << std::endl;
                if (mission_number == 2) {
                    std::cout << "Deploying blanket" << std::endl;
                }
            }


            drone.pose_setpoint_pub.publish(drone.get_setpoint());
        } else {
            drone.pose_setpoint_pub.publish(Vector3D{0}.to_pose());
        }

        // Deals with landing
        if (mission_number == desired_points.size()) {
            if ((drone.get_mavros_state().mode != "AUTO.LAND") &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (drone.set_mode_client.call(land_set_mode) &&
                    land_set_mode.response.mode_sent) {
                    ROS_INFO("Landing drone");
                    landing = true;
                    state_enter_time = ros::Time::now();  // landing enter time
                }

                last_request = ros::Time::now();
            }
        }

        if (landing and ros::Time::now() - state_enter_time > tolerance_time) {
            ROS_INFO("Landed drone");
            //std::cout << "Final energy: " << drone.get_energy() << std::endl;
            landed = true;
        }

        drone.calculate_energy();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
