#ifndef __HELPERS_HPP_
#define __HELPERS_HPP_
#pragma once

#include <Eigen/Eigen>
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include <cmath>
#include <string>

namespace helpers{
    inline void eigen_vector_to_msg(Eigen::VectorXd &vec, geometry_msgs::Vector3 &msg){
        msg.x = vec(0);
        msg.y = vec(1);
        msg.z = vec(2);
    }

    inline void eigen_vector_to_msg(Eigen::Vector3d &vec, geometry_msgs::Vector3 &msg){
        msg.x = vec(0);
        msg.y = vec(1);
        msg.z = vec(2);
    }

    inline void eigen_vector_to_msg(Eigen::Vector3d &vec, geometry_msgs::Point &msg){
        msg.x = vec(0);
        msg.y = vec(1);
        msg.z = vec(2);
    }

    inline void msg_to_eigen_vector(Eigen::Vector3d &vec, geometry_msgs::Vector3 &msg){
        vec(0) = msg.x;
        vec(1) = msg.y;
        vec(2) = msg.z;
    }

    inline void msg_to_eigen_vector(Eigen::Vector3d &vec, geometry_msgs::Point &msg){
        vec(0) = msg.x;
        vec(1) = msg.y;
        vec(2) = msg.z;
    }

    inline void tf_to_eigen_vector(Eigen::Vector3d &vec, tf::StampedTransform &transform){
        vec(0) = transform.getOrigin().x();
        vec(1) = transform.getOrigin().y();
        vec(2) = transform.getOrigin().z();
    }

    inline void tf_to_message(geometry_msgs::Pose &pose, tf::StampedTransform &transform){
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();
    }

    inline void reset_twist(geometry_msgs::Twist &in){
        in.angular.x = 0;
        in.angular.y = 0;
        in.angular.z = 0;
        in.linear.x = 0;
        in.linear.y = 0;
        in.linear.z = 0;
    }

    inline void reset_twist_angular(geometry_msgs::Twist &in){
        in.angular.x = 0;
        in.angular.y = 0;
        in.angular.z = 0;
    }

    inline double find_min_rotation(double target, double current){
        double yaw1, yaw2, yaw3;
        yaw1 = target - current;
        yaw2 = target - (current + 2 * M_PI);
        yaw3 = target - (current - 2 * M_PI);

        if(abs(yaw2) < abs(yaw1)) yaw1 = yaw2;
        if(abs(yaw3) < abs(yaw1)) yaw1 = yaw3;

        return yaw1;
    }

    inline Eigen::Matrix3d get_2d_rot_matrix(double rot_angle){
        Eigen::Matrix3d ret;
        ret <<  cos(rot_angle), -sin(rot_angle), 0,
                sin(rot_angle), cos(rot_angle), 0, 
                0, 0, 1;

        return ret;
    }

    inline bool get_current_position(tf::TransformListener* l, std::string world_frame, 
                                    std::string uav_frame, tf::StampedTransform& t, ros::Time time){
        try{
            l->waitForTransform(world_frame, uav_frame, time, ros::Duration(0.5));
            l->lookupTransform(world_frame, uav_frame, time, t);
            return true;
        }catch(tf::TransformException ex){
            ROS_ERROR("Failed to get world to uav tf: %s", ex.what());
            return false;
        }
    }
}

#endif