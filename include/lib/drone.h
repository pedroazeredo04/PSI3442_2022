#ifndef __DRONE_H__
#define __DRONE_H__

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

typedef enum q1_fsm_state {
    TAKEOFF,
    FOLLOW_TRAJECTORY,
    LANDING
} q1_fsm_state;

class Drone{
    public:
        // Constructors and Destructors
        Drone() = default;
        Drone(ros::NodeHandle nh);
        virtual ~Drone() = default;

        // Setters
        void set_setpoint(geometry_msgs::PoseStamped setpoint);
        void set_state(const mavros_msgs::State::ConstPtr& msg);
        void set_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

        // Getters
        geometry_msgs::PoseStamped get_pose();
        mavros_msgs::State get_mavros_state();
        geometry_msgs::PoseStamped get_setpoint();

        ros::Subscriber state_sub;
        ros::Subscriber pose_sub;
        ros::Publisher pose_setpoint_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;

        q1_fsm_state q1_state;

    private:
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped setpoint;
        mavros_msgs::State mavros_state;
};

#endif  // __DRONE_H__
