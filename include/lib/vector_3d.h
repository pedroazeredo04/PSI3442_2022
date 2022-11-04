#ifndef __VECTOR_3D_H__
#define __VECTOR_3D_H__

#include <geometry_msgs/PoseStamped.h>

class Vector3D {
    public:
        double x{0};
        double y{0};
        double z{0};

        Vector3D() = default;
        Vector3D(double x, double y, double z);
        virtual ~Vector3D() = default;

        geometry_msgs::PoseStamped to_pose();

        // Operators Overloads
        bool operator !=(const geometry_msgs::PoseStamped other_pose) const;

        bool operator ==(const geometry_msgs::PoseStamped other_pose) const;

    private:
        geometry_msgs::PoseStamped pose_msg;
};

#endif // __VECTOR_3D_H__
