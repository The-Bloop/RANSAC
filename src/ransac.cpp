#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#define PI 3.141593



class RANSAC 
{
private:
    // A ROS node
    ros::NodeHandle n;

    visualization_msgs::Marker points, line_strip, line_list;

    // Listen for scan messages
    ros::Subscriber scan_sub;

    // Publish position data
    ros::Subscriber pose_sub;
    double x = 0;

    double bot_angle;
    geometry_msgs::Point bot_pose;

public:

    ros::Publisher marker_pub;

    RANSAC() 
    {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        float f = 0.0;

        //Marker Creation
        line_strip.type = visualization_msgs::Marker::LINE_LIST;
        line_strip.id = 0;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "RANSAC";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.header.frame_id = "/base_link";

        //Publisher
        marker_pub = n.advertise<visualization_msgs::Marker>("ransac_vis", 10);

        // Subscribers
        scan_sub = n.subscribe("/base_scan", 1, &RANSAC::Scan_CallBack, this);
        pose_sub = n.subscribe("/base_pose_ground_truth", 1, &RANSAC::PoseChange_CallBack, this);
        
    }

    //Call Back Function for pose_sub
    void PoseChange_CallBack(const nav_msgs::Odometry & msg)
    {
        double q3 = msg.pose.pose.orientation.z;
        double q0 = msg.pose.pose.orientation.w;
        bot_angle = atan2((2*(q3*q0)), (1 - (2 * (q3*q3))));
        bot_pose.x = msg.pose.pose.position.x;
        bot_pose.y = msg.pose.pose.position.y;
        bot_pose.z = msg.pose.pose.position.z;
    }

    //Call Back Function for sacn_sub
    void Scan_CallBack(const sensor_msgs::LaserScan & msg)
    {
        line_strip.points.clear();

        int scanlength = (msg.angle_max - msg.angle_min) / msg.angle_increment;

        std::vector<int> scanIndexList = GetIndices(msg,scanlength);
        int limit = scanIndexList.size() * .05;

        int loopbreak = 0;
        
        if(scanIndexList.size() > 0)
        {
            while(scanIndexList.size() > 1 && loopbreak < 500)
            {
                loopbreak++;

                int length = scanIndexList.size();
                std::vector<int> pointIndicesList;
                std::vector<int> removeIndicesList;

                for(int k = 0;k < 50;k++)
                {
                    std::vector<int> tempList;
                    std::vector<int> tempRemoveList;

                    int p1index = rand() % length;
                    int p2index = rand() % length;

                    if(p1index != p2index)
                    {
                        double angle1 = bot_angle + (msg.angle_increment * (scanIndexList[p1index]));
                        geometry_msgs::Point p1 = CreatePoint(msg.ranges[scanIndexList[p1index]], angle1);

                        double angle2 = bot_angle + (msg.angle_increment * (scanIndexList[p2index]));
                        geometry_msgs::Point p2 = CreatePoint(msg.ranges[scanIndexList[p2index]], angle2);

                        geometry_msgs::Point l1 = CreateLine(p1,p2);
                        
                        //Getting Inliers
                        for(int n = 0;n < length; n++)
                        {
                            double angle3 = bot_angle + (msg.angle_increment * (scanIndexList[n]));
                            geometry_msgs::Point p3 = CreatePoint(msg.ranges[scanIndexList[n]], angle3);
                            geometry_msgs::Point l2 = CreateLine(p1,p3);
                            double dist = GetDistance(l1,l2);
                            if(dist <= 0.1)
                            {
                                tempList.push_back(scanIndexList[n]);
                                tempRemoveList.push_back(n);
                            }
                        }

                        //Getting list with most number of inliers
                        if(tempList.size() > pointIndicesList.size())
                        {
                            pointIndicesList = tempList;
                            removeIndicesList = tempRemoveList;
                        }
                    }
                }

                //Adding to Marker
                if(pointIndicesList.size() > 2)
                {
                    double angle5 = msg.angle_min + (msg.angle_increment * (pointIndicesList[0]));
                    geometry_msgs::Point p5 = CreatePoint(msg.ranges[pointIndicesList[0]], angle5);
                    line_strip.points.push_back(p5);

                    double angle6 = msg.angle_min + (msg.angle_increment * (pointIndicesList[pointIndicesList.size() - 1]));
                    geometry_msgs::Point p6 = CreatePoint(msg.ranges[pointIndicesList[pointIndicesList.size() - 1]], angle6);                     
                    line_strip.points.push_back(p6); 
                }

                //Removing all inliers from the main list
                for(int j = removeIndicesList.size() - 1;j >= 0; j--)
                {
                    auto it1 = scanIndexList.begin();
                    std::advance(it1, removeIndicesList[j]);
                    scanIndexList.erase(it1);
                }
            }

            line_strip.scale.x = 0.03;
            line_strip.scale.y = 0.03;
            line_strip.color.g = 1.0f;
            line_strip.color.a = 1.0;
            marker_pub.publish(line_strip);
        }       
    }

    //Get Indices of LaserScan ranges that are hitting obstacle
    std::vector<int> GetIndices(const sensor_msgs::LaserScan & msg, int length)
    {
        std::vector<int> indexList;
        for(int i = 0;i<length;i++)
        {
            if(msg.ranges[i] < msg.range_max)
            {
                indexList.push_back(i);
            }
        }
        return indexList;
    }

    //Create Point with LaserScan range and angle
    geometry_msgs::Point CreatePoint(double range, double angle)
    {
        geometry_msgs::Point point;
        point.x =  range * cos(angle);
        point.y =  range * sin(angle);
        point.z = 0;
        return point;
    }

    //Create Line with 2 points
    geometry_msgs::Point CreateLine(geometry_msgs::Point& point0, geometry_msgs::Point& point1)
    {
        geometry_msgs::Point line;
        line.x = point0.x + (point1.x - point0.x);
        line.y = point0.y + (point1.y - point0.y);
        line.z = 0;
        return line;
    }

    //Get distance between 2 points
    double GetDistance(geometry_msgs::Point& line1, geometry_msgs::Point& line2)
    {
        double modl1xl2 = sqrt(pow(((line2.x*line1.y) - (line2.y*line1.x)) , 2));
        double modl2 = sqrt(pow(line1.x,2) + pow(line1.y,2));
        double dist = modl1xl2/modl2;
        return dist;
    }

}; // end of class definition


int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "perception");
    RANSAC r;
    ros::spin();
    return 0;
}

