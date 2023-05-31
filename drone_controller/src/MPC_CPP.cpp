#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

// Set the maximum velocity that can be commanded
double vel_max = 2.0;

// Set the time step of the controller
double dt = 0.1;

// Define the cost function parameters
double A = 1.0;
double C = 100.0;

// Set the prediction horizon and the number of steps to simulate
int N = 10;
int N_sim = 100;

// Set the initial position and velocity
geometry_msgs::PoseStamped current_pos;
geometry_msgs::TwistStamped current_vel;

// Set the reference position and velocity
geometry_msgs::PoseStamped ref_pos;
geometry_msgs::TwistStamped ref_vel;

// Set the previous velocity command
geometry_msgs::TwistStamped prev_vel;

// Set the current controller state
int current_state = 0;

// Define the MPC controller function
geometry_msgs::TwistStamped mpc_controller()
{
    // Initialize the minimum cost and the corresponding velocity command
    double min_cost = 1e9;
    geometry_msgs::TwistStamped vel_cmd_min;

    // Loop through all possible velocity commands up to the maximum velocity
    for (double v = 0.0; v <= vel_max; v += 0.1)
    {
        // Initialize the predicted position and velocity
        geometry_msgs::PoseStamped pred_pos = current_pos;
        geometry_msgs::TwistStamped pred_vel = current_vel;

        // Simulate the system for the prediction horizon
        for (int i = 0; i < N; i++)
        {
            // Calculate the control input
            double ax = A * (ref_pos.pose.position.x - pred_pos.pose.position.x);
            double ay = A * (ref_pos.pose.position.y - pred_pos.pose.position.y);
            double az = A * (ref_pos.pose.position.z - pred_pos.pose.position.z);
            double vx = v * cos(ref_pos.pose.orientation.yaw);
            double vy = v * sin(ref_pos.pose.orientation.yaw);
            double vz = (ref_pos.pose.position.z - pred_pos.pose.position.z) / dt + az * dt;
            geometry_msgs::TwistStamped vel_cmd;
            vel_cmd.twist.linear.x = vx;
            vel_cmd.twist.linear.y = vy;
            vel_cmd.twist.linear.z = vz;

            // Apply the control input to the system dynamics
            pred_vel.twist.linear.x += ax * dt;
            pred_vel.twist.linear.y += ay * dt;
            pred_vel.twist.linear.z += az * dt;
            pred_pos.pose.position.x += pred_vel.twist.linear.x * dt;
            pred_pos.pose.position.y += pred_vel.twist.linear.y * dt;
            pred_pos.pose.position.z += pred_vel.twist.linear.z * dt;

            // Calculate the cost
            double cost = C * (ref_vel.twist.linear.x - pred_vel.twist.linear.x) *
                (ref_vel.twist.linear.x - pred_vel.twist.linear.x) +
                C * (ref_vel.twist.linear.y - pred_vel.twist.linear.y) *
                (ref_vel.twist.linear.y - pred_vel.twist.linear.y) +
                C * (ref_vel.twist.linear.z - pred_vel.twist.linear.z) *
                (ref_vel.twist.linear.z - pred_vel.twist.linear.z) +
		(vel_cmd.twist.linear.x - prev_vel.twist.linear.x) *
		(vel_cmd.twist.linear.x - prev_vel.twist.linear.x) +
		(vel_cmd.twist.linear.y - prev_vel.twist.linear.y) *
		(vel_cmd.twist.linear.y - prev_vel.twist.linear.y) +
		(vel_cmd.twist.linear.z - prev_vel.twist.linear.z) *
		(vel_cmd.twist.linear.z - prev_vel.twist.linear.z);

        // Update the minimum cost and corresponding velocity command
        if (cost < min_cost)
        {
            min_cost = cost;
            vel_cmd_min = vel_cmd;
        }
    }
}

// Update the previous velocity command
prev_vel = vel_cmd_min;

// Return the velocity command with minimum cost
return vel_cmd_min;
}

// Define the state callback function
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
// Set the current state based on the received message
if (msg->mode == "OFFBOARD" && msg->armed)
{
current_state = 1;
}
else
{
current_state = 0;
}
}

// Define the pose callback function
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
// Update the current position
current_pos = *msg;
}

// Define the velocity callback function
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
// Update the current velocity
current_vel = *msg;
}

int main(int argc, char **argv)
{
// Initialize the ROS node
ros::init(argc, argv, "mpc_controller");
ros::NodeHandle nh;
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, vel_cb);

// Advertise the necessary topics
ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

// Service clients for arming, setting mode, and taking off
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

// Set the initial reference position and velocity
ref_pos.pose.position.x = 0.0;
ref_pos.pose.position.y = 0.0;
ref_pos.pose.position.z = 2.0;
ref_pos.pose.orientation.w = 1.0;
ref_vel.twist.linear.x = 0.0;
ref_vel.twist.linear.y = 0.0;
ref_vel.twist.linear.z = 0.0;

// Set the loop rate
ros::Rate loop_rate(1/dt);

// Wait for the connection to be established
while (ros::ok() && !current_state)
{
    ros::spinOnce();
    loop_rate.sleep();
}

// Arm the vehicle
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;
if (!arming_client.call(arm_cmd) || !arm_cmd.response.success)
{
ROS_ERROR("Failed to arm the vehicle");
return -1;
}

ROS_INFO("Vehicle armed");

// Set the flight mode to OFFBOARD
mavros_msgs::SetMode offboard_set_mode;
offboard_set_mode.request.custom_mode = "OFFBOARD";
if (!set_mode_client.call(offboard_set_mode) || !offboard_set_mode.response.success)
{
ROS_ERROR("Failed to set flight mode to OFFBOARD");
return -1;
}

ROS_INFO("Flight mode set to OFFBOARD");

// Takeoff to a height of 2 meters
mavros_msgs::CommandTOL takeoff_cmd;
takeoff_cmd.request.altitude = 2.0;
if (!takeoff_client.call(takeoff_cmd) || !takeoff_cmd.response.success)
{
ROS_ERROR("Failed to takeoff");
return -1;
}

ROS_INFO("Vehicle in air at 2 meters");

// Loop at 10 Hz and publish the velocity commands
ros::Rate rate(10.0);
while (ros::ok())
{
// If the vehicle is not armed or in OFFBOARD mode, do not publish velocity commands
if (!current_state)
{
ros::spinOnce();
rate.sleep();
continue;
}
// Compute the velocity command using the MPC controller
geometry_msgs::TwistStamped vel_cmd = mpc_controller();

// Publish the velocity command
vel_cmd.header.stamp = ros::Time::now();
vel_cmd_pub.publish(vel_cmd);

// Update the previous velocity command
prev_vel = vel_cmd;

ros::spinOnce();
rate.sleep();
}

// Land the vehicle
mavros_msgs::CommandTOL land_cmd;
if (!land_client.call(land_cmd) || !land_cmd.response.success)
{
ROS_ERROR("Failed to land");
return -1;
}

ROS_INFO("Vehicle landed");

// Disarm the vehicle
mavros_msgs::CommandBool disarm_cmd;
disarm_cmd.request.value = false;
if (!arming_client.call(disarm_cmd) || !disarm_cmd.response.success)
{
ROS_ERROR("Failed to disarm the vehicle");
return -1;
}

ROS_INFO("Vehicle disarmed");

// Return success
return 0;
}
