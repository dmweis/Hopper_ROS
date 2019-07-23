#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/AbstractOcTree.h"
#include "octomap/math/Vector3.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>

using namespace std;
using namespace octomap;

tf2_ros::Buffer *globalTfBuffer;
tf2_ros::TransformListener *globalTfListener;
ros::Publisher *global_marker_pub;
ros::Publisher *global_collision_pub;
int globalMarkerCounter;

void displayMarker(octomath::Vector3 *vector)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time();
    marker.ns = "obstacle_markers";
    marker.id = globalMarkerCounter++;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x = vector->x();
    marker.pose.position.y = vector->y();
    marker.pose.position.z = vector->z();
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(10);

    global_marker_pub->publish(marker);
}

void deleteMarker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time();
    marker.ns = "obstacle_markers";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETEALL;

    global_marker_pub->publish(marker);
}

void octomapCallback(const octomap_msgs::Octomap msg)
{
    // ROS_INFO("Hi [%s]", msg.header.frame_id.c_str());
    octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(msg);
    octomap::OcTree *octTree = dynamic_cast<octomap::OcTree *>(tree);
    if (octTree)
    {
        geometry_msgs::TransformStamped transformStamped;

        do
        {
            try
            {
                transformStamped = globalTfBuffer->lookupTransform("odom", "body_link", ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        } while (ros::ok());
        float x = transformStamped.transform.translation.x;
        float y = transformStamped.transform.translation.y;
        float z = transformStamped.transform.translation.z;

        octomath::Vector3 position = octomath::Vector3(x, y, z);

        float qx = transformStamped.transform.rotation.x;
        float qy = transformStamped.transform.rotation.y;
        float qz = transformStamped.transform.rotation.z;
        float qu = transformStamped.transform.rotation.w;

        octomath::Quaternion rotation(qu, qx, qy, qz);

        octomath::Vector3 forward = octomath::Vector3(1, 0, 0);
        octomath::Vector3 orientation = rotation.rotate(forward);

        octomath::Vector3 heading(orientation.x(), orientation.y(), 0);
        octomath::Vector3 result = octomath::Vector3(0, 0, 0);

        bool pointsDetected = false;
        sensor_msgs::PointCloud resultCloud;
        resultCloud.header.frame_id = "odom";
        resultCloud.header.stamp = ros::Time();
        for (float height = 0.0; height <= 0.2; height += 0.01)
        {
            octomath::Vector3 origin(position.x(), position.y(), height);
            if (octTree->castRay(origin, heading, result, true, 10))
            {
                pointsDetected = true;
                ROS_INFO("Found thingy!");
                OcTreeNode *hitNode = octTree->search(result);
                if (hitNode)
                {
                    // std::cout << "Value: " << hitNode->getValue() << "\n";
                    displayMarker(&result);
                    geometry_msgs::Point32 hitCoordinates;
                    hitCoordinates.x = result.x();
                    hitCoordinates.y = result.y();
                    hitCoordinates.z = result.z();
                    resultCloud.points.push_back(hitCoordinates);

                    sensor_msgs::ChannelFloat32 channelData;
                    channelData.name = "intensity";
                    channelData.values.push_back(hitNode->getValue());
                    resultCloud.channels.push_back(channelData);
                }
            }
        }
        if (pointsDetected)
        {
            global_collision_pub->publish(resultCloud);
        }
        else
        {
            deleteMarker();
        }

        // double v = 0;
        // int count = 0;
        // for (OcTree::leaf_iterator it = octTree->begin_leafs(),
        //                            end = octTree->end_leafs();
        //      it != end; ++it)
        // {
        //     count++;
        //     //manipulate node, e.g.:
        //     // std::cout << "Node size: " << it.getSize() << std::endl;
        //     // std::cout << "Node value: " << it->getValue() << std::endl;
        //     if (it->getOccupancy())
        //     {
        //         std::cout << "Node center: " << it.getCoordinate() << std::endl;
        //         cout << "occupied" << endl;
        //     }
        //     v = v + (pow(it.getSize(), 3));
        // }

        // std::cout<<"VOLUME::::"<<v<<endl;
        // std::cout << count <<endl;
        delete octTree;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    globalTfBuffer = &tfBuffer;
    tf2_ros::TransformListener tfListener(*globalTfBuffer);
    globalTfListener = &tfListener;

    ros::Subscriber sub = n.subscribe("/global/octomap_full", 1000, octomapCallback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("hopper/obstacle_detected/markers", 10);
    ros::Publisher collisionPublisher = n.advertise<sensor_msgs::PointCloud>("hopper/detected_obstacles", 10);

    global_collision_pub = &collisionPublisher;
    global_marker_pub = &marker_pub;

    ros::spin();

    return 0;
}