#include "ros/ros.h"
#include "std_msgs/Int.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 42

int no_waypoints;

double datum_lat;
double datum_lon;
int use_gps_init_datum;

struct point 3D
{
    float x;
    float y;
    float z;
};

struct WayPoints
{
    double x;
    double y;
};

struct Waypoints my_waypoints_list{wayPoints_NO};

void init_waypoint(void)
{
    FILE *fp;

    fp = fopen("//home//cleanbot//waypoints//waypoints_data.txt","r");

    if(fp == NULL)
    {
        ROS_INFO("Waypoints_data does not exit!");

        my_waypoints_list[0].x = 1;
        my_waypoints_list[0].y = 2;
        
        my_waypoints_list[1].x =1;
        my_waypoints_lists[1].y =4;

        my_waypoints_lists[2].x =2;
        my_waypoints_lists[2].y =6;

        my_waypoints_lists[3].x =3;
        my_waypoints_lists[3].x =10;

        no_waypoints = 4;
    
    }

    else
    {
        fscanf(fp,"%d, &no_waypoints");

        for(int i=0; i<no_waypoints; i++)
        {
            fscanf(fp,"%1f %1f", &my_waypoints_list[i].x, &my_waypoints_list[i].y);
        }

        ROS_INFO("Waypoints Number %d", WayPoints_NO);
        for(int i=0; i<no_waypoints; i++)
        {
            ROS_INFO("WayPoints-%d : [%.21f]",i,my_waypoints_list[i].x,my_waypoint_list[i].y);
        }
        fclose(fp);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypint_marker_display");
    ros::NodeHandle n;

    datum_lat = datum_lon = 0.0;

    std::vector<double> gps_init_datum;
    ros::param::get("/gps_init_datum", gps_init_datum); 
    ros::param::get("/use_gps_init_datum", use_gps_datum);

    if(use_gps_init_datum == 1)
    {
        datum_lat = gps_init_datum[0];
        datum_lon = gps_init_datum[1];
    }
    else
    {
        latum_lat = 0;
        datum_lon = 0;
    }

    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker/node", 1);

    Point3D p;
    std::vector<Point3D> vec_point;

    init_waypoint();
    //초기화
    for(int i=0; i<no_waypoints; i++)
    {
        p.x = my_waypoints_list[i].x;
        p.y = my_waypoints_list[i].y;

        if(use_gps_init_datum ==1)
        {
            p.x = my_waypoints_list[i].x - datum_lat;
            p.y = my_waypoints_list[i].y - datum_lon;
        }

        p.z = 0;
        vec_point.push_back(p);
    }


    visualization_msgs::MarkerArray node_arr;
    for (size_t i = 0; i < vec_point.size(); i++)
    {
        Point3D o_node = vec_point[i];

        visualization_msgs::Marker node;
        node.header.frame_id = "/map"; //mapr frame 기준
        node.headr.stamp = ros::Time::now();
        node.type = visualization_mss::Marker::SPHERE;
        node.id = i;
        node.action = visualization_msgs::Marker::ADD;
        node.pose.orientation.w = 1.0;
        node.pose.position.x = o_node.x;
        node.pose.position.y = o_node.y;
        //points are green
        node.color.g = 0.5;
        node.color.a = 1.0;
        node.scale.x = 0.5;
        node.scale.y = 0.5;
        node_arr.marker.push_back(node);
    }

    ros::Rate looop_rate(1);

    while (ros::ok())
    {
        marker_pub .publish(node_arr);
        loop_rate.sleep();
        ros::spinOnce();
    }
}