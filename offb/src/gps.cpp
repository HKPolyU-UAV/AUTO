#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "include/movement.h"

using namespace std;
enum flying_step
{
    IDLE,
    TAKEOFF,
    HOVER,
    LAND,
    SWAY,
    MOVE,
    END,
};

static flying_step fly = IDLE;
static UAVpose uavinfo;
static double ax, ay, az;

static mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo.x = pose->pose.position.x;
    uavinfo.y = pose->pose.position.y;
    uavinfo.z = pose->pose.position.z;
    uavinfo.ow = pose->pose.orientation.w;
    uavinfo.ox = pose->pose.orientation.x;
    uavinfo.oy = pose->pose.orientation.y;
    uavinfo.oz = pose->pose.orientation.z;
}

void rpy_to_Q(double yaw, double &w, double &x, double &y, double &z)
{
    w = cos(0) * cos (0) * cos (yaw/2) + sin (0) * sin (0) * sin (yaw/2) ;
    x = sin(0) * cos (0) * cos (yaw/2) - cos (0) * sin (0) * sin (yaw/2) ;
    y = cos(0) * sin (0) * cos (yaw/2) + sin (0) * cos (0) * sin (yaw/2) ;
    z = cos(0) * cos (0) * sin (yaw/2) - sin (0) * sin (0) * cos (yaw/2) ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber uav_pos_sub =nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",1,position_callback);


    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    double last_request = ros::Time::now().toSec();

    vector<waypts> initialize;
    waypts hi1 = {0,0,0}, hi2 = {0,0,2};
    initialize.push_back(hi1);
    initialize.push_back(hi2);

    movement move(initialize);

    cout<<uavinfo.x<<endl;
    cout<<uavinfo.y<<endl;
    cout<<uavinfo.z<<endl;
    waypts test = {2,0,2};
    double t = 1;
    double yaw = 0;

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now().toSec() - last_request > ros::Duration(5.0).toSec()))
          {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now().toSec();
          }
        else
          {
            if( !current_state.armed && (ros::Time::now().toSec() - last_request > ros::Duration(5.0).toSec()))
              {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                  {
                    ROS_INFO("Vehicle armed");
                    fly = TAKEOFF;
                    last_request = ros::Time::now().toSec();
                  }
                last_request = ros::Time::now().toSec();
              }
           }

        if(fly == TAKEOFF)
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), hi1, hi2);
            if(move.switchflymode_ == true)
            {
                fly = HOVER;
                last_request = ros::Time::now().toSec();
            }
        }

        if(fly == HOVER)
        {
            move.hover(pose, hi2);
            if(ros::Time::now().toSec() - last_request >= ros::Duration(4.0).toSec())
            {
                fly = MOVE;
                last_request = ros::Time::now().toSec();
            }
        }

        if(fly == MOVE)
        {
            yaw = 0;
            rpy_to_Q(yaw, pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
            if(ros::Time::now().toSec() - last_request >= ros::Duration(10.0).toSec())
            {
                fly = SWAY;
                last_request = ros::Time::now().toSec();
            }
        }

        if(fly == SWAY)
        {
            yaw += 0.01;
            rpy_to_Q(yaw, pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
            if(ros::Time::now().toSec() - last_request >= ros::Duration(15.0).toSec())
            {
                fly = LAND;
                last_request = ros::Time::now().toSec();
            }
        }


        if(fly == LAND)
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), hi2, hi1);
            if(move.switchflymode_)
                fly = END;
        }


        if(fly == END)
        {
            pose.pose.position.z = pose.pose.position.z - 0.02;

            if(pose.pose.position.z < 0.05)
            {
                arm_cmd.request.value = false;
                if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success )
                {
                    cout << "UAV about to touch ground" << endl;
                    cout << "Touched and end...."<< endl;
                    return 0;//break the control UAV will land automatically
                }
             }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


