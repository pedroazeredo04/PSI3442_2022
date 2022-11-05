#ifndef __DRONE_H__
#define __DRONE_H__

#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>

class Drone {
    public:
        // Constructors and Destructors
        Drone() = default;
        Drone(ros::NodeHandle nh);
        virtual ~Drone() = default;

        // Setters
        void set_setpoint(geometry_msgs::PoseStamped setpoint);

        void set_state(const mavros_msgs::State::ConstPtr& msg);

        void set_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void set_battery(const sensor_msgs::BatteryState::ConstPtr& msg);

        void calculate_energy();

        // Getters
        geometry_msgs::PoseStamped get_pose();

        mavros_msgs::State get_mavros_state();

        geometry_msgs::PoseStamped get_setpoint();

        double get_energy();

        ros::Subscriber state_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber battery_sub;
        ros::Subscriber velocity_sub;
        ros::Publisher pose_setpoint_pub;
        ros::Publisher vel_setpoint_pub;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient arming_client;

    private:
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped setpoint;
        mavros_msgs::State mavros_state;
        sensor_msgs::BatteryState battery;
        double energy;
};

#endif // __DRONE_H__
