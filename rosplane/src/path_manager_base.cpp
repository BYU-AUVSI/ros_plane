#include "path_manager_base.h"
#include "path_manager_example.h"

namespace rosplane
{

    path_manager_base::path_manager_base() :
            nh_(ros::NodeHandle()), /** nh_ stuff added here */
            nh_private_(ros::NodeHandle("~"))
    {
        nh_private_.param<double>("R_min", params_.R_min, 25.0);
        nh_private_.param<double>("update_rate", update_rate_, 10.0);

        vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
        new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);
        current_path_pub_ = nh_.advertise<rosplane_msgs::Current_Path>("current_path", 10);
        extended_path_pub_ = nh_.advertise<rosplane_msgs::Extended_Path>("extended_path", 10);

        update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &path_manager_base::current_path_publish,
                                        this);

        num_waypoints_ = 0;

        state_init_ = false;
    }

    void path_manager_base::vehicle_state_callback(const rosplane_msgs::StateConstPtr &msg)
    {
        vehicle_state_ = *msg;

        state_init_ = true;
    }

    void path_manager_base::new_waypoint_callback(const rosplane_msgs::Waypoint &msg)
    {
        if (msg.clear_wp_list == true)
        {
            waypoints_.clear();
            num_waypoints_ = 0;
            last_waypoint_index_ = 0;
            return;
        }
        if (msg.set_current || num_waypoints_ == 0)
        {
            waypoint_s currentwp;
            currentwp.position[0] = vehicle_state_.position[0];
            currentwp.position[1] = vehicle_state_.position[1];
            currentwp.position[2] = (vehicle_state_.position[2] > -25 ? msg.w[2] : vehicle_state_.position[2]);
            currentwp.course_command = vehicle_state_.chi;
            currentwp.course_valid = msg.chi_valid;
            currentwp.airspeed_command = msg.Va_d;

            waypoints_.clear();
            waypoints_.push_back(currentwp);
            num_waypoints_ = 1;
            last_waypoint_index_ = 0;
        }
        waypoint_s nextwp;
        nextwp.position[0] = msg.w[0];
        nextwp.position[1] = msg.w[1];
        nextwp.position[2] = msg.w[2];
        nextwp.course_command = msg.chi_d;
        nextwp.course_valid = msg.chi_valid;
        nextwp.airspeed_command = msg.Va_d;
        waypoints_.push_back(nextwp);
        num_waypoints_++;
    }

    void path_manager_base::current_path_publish(const ros::TimerEvent &)
    {

        struct input_s input;
        input.position_north = vehicle_state_.position[0];               /** position north */
        input.position_east = vehicle_state_.position[1];               /** position east */
        input.altitude = -vehicle_state_.position[2];                /** altitude */
        input.course_rad = vehicle_state_.chi;

        struct output_s output;

        if (state_init_ == true)
        {
            manage(params_, input, output);
        }
        if(output == last_output)
            return;
        last_output = output;

        rosplane_msgs::Current_Path current_path;
        rosplane_msgs::Extended_Path extended_path;

        if (output.is_straight)
        {
            current_path.path_type = current_path.LINE_PATH;
            extended_path.path_type = current_path.LINE_PATH;
        } else
        {
            current_path.path_type = current_path.ORBIT_PATH;
            extended_path.path_type = extended_path.ORBIT_PATH;
        }
        current_path.Va_d = output.airspeed_command;
        extended_path.Va_d = output.airspeed_command;
        for (int i = 0; i < 3; i++)
        {
            current_path.r[i] = output.line_origin[i];
            current_path.q[i] = output.line_direction[i];
            current_path.c[i] = output.center[i];
            extended_path.r[i] = output.line_origin[i];
            extended_path.q[i] = output.line_direction[i];
            extended_path.c[i] = output.center[i];
        }
        current_path.rho = output.orbit_radius;
        current_path.lambda = output.orbit_direction;
        extended_path.rho = output.orbit_radius;
        extended_path.lambda = output.orbit_direction;

        extended_path.line_end[0] = output.line_end[0];
        extended_path.line_end[1] = output.line_end[1];
        extended_path.line_end[2] = output.line_end[2];

        current_path_pub_.publish(current_path);
        extended_path_pub_.publish(extended_path);
    }

    bool operator==(const path_manager_base::output_s &o1, const path_manager_base::output_s &o2)
    {
        if (o1.is_straight != o2.is_straight ||
            o1.airspeed_command != o2.airspeed_command ||
            o1.orbit_radius != o2.orbit_radius ||
            o1.orbit_direction != o2.orbit_direction)
            return false;
        for (int i = 0; i < 3; i++)
            if (o1.line_origin[i] != o2.line_origin[i] ||
                o1.line_end[i] != o2.line_end[i] ||
                o1.line_direction[i] != o2.line_direction[i] ||
                o1.center[i] != o2.center[i])
                return false;
        return true;
    }
} //end namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosplane_path_manager");
    rosplane::path_manager_base *est = new rosplane::path_manager_example();

    ros::spin();

    return 0;
}
