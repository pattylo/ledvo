#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_message/FSMInfo.h>
#include <airo_message/TakeoffLandTrigger.h>
#include <airo_message/Reference.h>

static geometry_msgs::PoseStamped local_pose;
static airo_message::FSMInfo fsm_info;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

enum TrajCatog{
    CIRCLE,
    SQUARE,
    LEMNISCATE
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void fsm_info_cb(const airo_message::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    // sub and pub
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_message::FSMInfo>("/airo_control/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_message::Reference>("/airo_control/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",10);

    // sub and pub objects
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    airo_message::Reference target_pose;
    airo_message::TakeoffLandTrigger takeoff_land_trigger;

    // traj object
    int TrajType;
    nh.getParam("/uav_path/traj_type",TrajType);

    // circle
    double frequency;
    double radius;
    double x0, y0, z0;
    
    nh.getParam("/uav_path/freq", frequency);     
        // rad_vel =  2 * pi / t = 2 * pi * freq. 
    nh.getParam("/uav_path/radius", radius);
    nh.getParam("/uav_path/x0", x0);
    nh.getParam("/uav_path/y0", y0);
    nh.getParam("/uav_path/z0", z0);


        
    // square
    double x_edge;
    double y_edge;
    double z_edge;
    //
    

    int circle_step_i = 0;




    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(fsm_info.is_landed == true){
                    while(ros::ok()){
                        takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                        takeoff_land_trigger.header.stamp = ros::Time::now();
                        takeoff_land_pub.publish(takeoff_land_trigger);
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        if(fsm_info.is_waiting_for_command){
                            state = COMMAND;
                            break;
                        }
                    }
                }
                break;
            }

            case COMMAND:{
                if(fsm_info.is_waiting_for_command)
                {
                    switch (TrajType)
                    {
                    case CIRCLE:
                        /* code */
                        traj[:,0] = -r*np.cos(t*v/r)+x0
                        traj[:,1] = -r*np.sin(t*v/r)+y0
                        traj[:,2] = z0

                        // tangent_vel = 2 pi r / duration
                        // tangel_vel * dt / r = angular location

                        double theta = ;
                        double x = -radius * cos(theta) + x0;
                        double y = radius * sin(theta);
                        double z = 2.0 * radius * sin(theta / 2.0); // Adjust the multiplier as needed
                        break;

                    case SQUARE:
                        /* code */
                        break;
                        
                    case LEMNISCATE:
                        /* code */
                        break;
                    
                    default:
                        break;
                    }

                    target_pose.header.stamp = ros::Time::now();
                    command_pub.publish(target_pose);                    
                }
                break;
            }

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}