/**
 * @file path_manager_base.h
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for RosPlane software
 */

#ifndef PATH_MANAGER_BASE_H
#define PATH_MANAGER_BASE_H

#include <ros/ros.h>
#include <rosplane_msgs/State.h>
#include <rosplane_msgs/Current_Path.h>
#include <rosplane_msgs/Extended_Path.h>
#include <rosplane_msgs/Waypoint.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/FluidPressure.h>
#include <math.h>
#include <Eigen/Eigen>
#include <rosplane/ControllerConfig.h>

namespace rosplane
{
    class path_manager_base
    {
    public:
        path_manager_base();
        struct output_s
        {
            bool is_straight;             /** Inicates strait line or orbital path (true is line, false is orbit) */
            float airspeed_command;             /** Desired airspeed (m/s) */
            float line_origin[3];             /** Vector to origin of straight line path (m) */
            float line_end[3];
            float line_direction[3];             /** Unit vector, desired direction of travel for line path */
            float center[3];             /** Center of orbital path (m) */
            float orbit_radius;              /** Radius of orbital path (m) */
            int8_t orbit_direction;          /** Direction of orbital path (cw is 1, ccw is -1) */
        };
    protected:

        struct waypoint_s
        {
            float position[3];
            float course_command;
            bool course_valid;
            float airspeed_command;
        };

        std::vector<waypoint_s> waypoints_;
        int num_waypoints_;
        int last_waypoint_index_;                 /** index to the waypoint that was most recently achieved */

        struct input_s
        {
            float position_north;               /** position north */
            float position_east;               /** position east */
            float altitude;                /** altitude */
            float course_rad;              /** course angle */
        };

        output_s last_output;

        struct params_s
        {
            double R_min;
        };

        virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

    private:

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber vehicle_state_sub_;     /**< vehicle state subscription */
        ros::Subscriber new_waypoint_sub_;      /**< new waypoint subscription */
        ros::Publisher current_path_pub_;      /**< controller commands publication */
        ros::Publisher extended_path_pub_;

        struct params_s params_;

        rosplane_msgs::State vehicle_state_;     /**< vehicle state */

        double update_rate_;
        ros::Timer update_timer_;

        void vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg);
        bool state_init_;
        void new_waypoint_callback(const rosplane_msgs::Waypoint &msg);
        void current_path_publish(const ros::TimerEvent &);
    };

    bool operator==(const path_manager_base::output_s &o1, const path_manager_base::output_s &o2);
} //end namespace
#endif // PATH_MANAGER_BASE_H
