#include "include/movement.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <istream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;
static visualization_msgs::Marker uav_points;
static visualization_msgs::Marker poo_points;
static vector<UAVpose> traj;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual");
    ros::NodeHandle nh;
    ros::Publisher rviz_visual = nh.advertise <visualization_msgs::Marker>("gt_points",1);
    ifstream in;
    in.open("/home/patrick/track_ws/src/offb/src/vexp7/mav_pos.txt");
    string line;
    int counter_per_file = 1;


    if (in.is_open())
    {
        UAVpose temp;
        while (getline(in, line)) //get 1 row as a string
        {
            //cout<<line<<endl;
            switch (counter_per_file % 8)
            {
            case 2:
                temp.x = stod(line);
                break;
            case 3:
                temp.y = stod(line);
                break;
            case 4:
                temp.z = stod(line);
                break;
            case 5:
                temp.ow = stod(line);
                break;
            case 6:
                temp.ox = stod(line);
                break;
            case 7:
                temp.oy = stod(line);
                break;
            case 0:
                temp.oz = stod(line);
                traj.push_back(temp);
                break;
            default:
                break;
            }
            counter_per_file++;
//            if(counter_per_file > 100)
//                break;
        }
//        cout<<endl;
//        cout<<traj.size()<<endl;
//        for(auto stuff : traj)
//        {
//            cout<<stuff.x<<endl;
//        }

    }
    else
        cout<<"failed to open"<<endl;

    cout<<traj.size()<<endl;

    visualization_msgs::Marker points;
    points.header.frame_id     = "/map";
    points.header.stamp        = ros::Time::now();
    points.ns                  = "wp_point";
    points.action              = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;
    points.pose.orientation.x  = 0.0;
    points.pose.orientation.y  = 0.0;
    points.pose.orientation.z  = 0.0;
    points.id                  = 1;
    points.type                = visualization_msgs::Marker::SPHERE_LIST;
    points.scale.x = 0.03;
    points.scale.y = 0.03;
    points.scale.z = 0.03;
    points.color.a = 1.0;
    points.color.r = 1.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

//    for(auto what : traj)
//    {
//        geometry_msgs::Point p;
//        p.x = what.x;
//        p.y = what.y;
//        p.z = what.z;
//        points.points.push_back(p);
//    }
    size_t i=700;
    ros::Rate rate(50);
    while(ros::ok())
    {
        if(i < 2400)
        {
            if(i % 40 == 0)
            {
                geometry_msgs::Point p;
                p.x = traj[i].x;
                p.y = traj[i].y;
                p.z = traj[i].z;
                points.points.push_back(p);
            }

        }
        i++;
        cout<<i<<endl;
        rviz_visual.publish(points);
        rate.sleep();
        //rviz_visual.publish(poo_points);
    }

    return 0;
}
