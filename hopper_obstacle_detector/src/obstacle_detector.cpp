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
#include <hopper_obstacle_detector/CastRay.h>

#include <geometry_msgs/Vector3Stamped.h>

using namespace std;
using namespace octomap;
using namespace octomath;

tf2_ros::Buffer *globalTfBuffer;
tf2_ros::TransformListener *globalTfListener;
ros::Publisher *global_marker_pub;
ros::Publisher *global_collision_pub;
ros::Publisher *global_obstacle_edge_pub;
int globalMarkerCounter;

octomap::OcTree *last_octmap_cloud = NULL;

void getTransform(const char *from_frame, const char *to_frame, Vector3 &position, Quaternion &rotation)
{
    geometry_msgs::TransformStamped transformStamped;
    do
    {
        try
        {
            transformStamped = globalTfBuffer->lookupTransform(from_frame, to_frame, ros::Time(0));
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

    position = Vector3(x, y, z);

    float qx = transformStamped.transform.rotation.x;
    float qy = transformStamped.transform.rotation.y;
    float qz = transformStamped.transform.rotation.z;
    float qu = transformStamped.transform.rotation.w;

    rotation = Quaternion(qu, qx, qy, qz);
}

bool castRay(hopper_obstacle_detector::CastRay::Request &req, hopper_obstacle_detector::CastRay::Response &res)
{
    for (int i = 0; i < req.origins.size(); i++)
    {
        Vector3 frame_translation;
        Quaternion rotation;
        getTransform("odom", req.origins[i].header.frame_id.c_str(), frame_translation, rotation);

        Vector3 direction(req.directions[i].x, req.directions[i].y, req.directions[i].z);
        direction = rotation.rotate(direction);

        Vector3 position(req.origins[i].vector.x, req.origins[i].vector.y, req.origins[i].vector.z);
        position = rotation.rotate(position);
        position += frame_translation;

        Vector3 result;

        geometry_msgs::Vector3Stamped resultHit;
        resultHit.header.frame_id = req.origins[i].header.frame_id;
        bool hitDetected = false;
        cout << "cating" << endl;
        cout << frame_translation << endl;
        if (last_octmap_cloud->castRay(position, direction, result, true, 10))
        {
            cout << "cast worked" << endl;

            OcTreeNode *hitNode = last_octmap_cloud->search(result);
            if (hitNode)
            {
                hitDetected = true;
                Vector3 translatedResult;
                translatedResult = result - frame_translation;
                Quaternion invertedRotation = rotation.inv();
                translatedResult = invertedRotation.rotate(translatedResult);
                resultHit.vector.x = translatedResult.x();
                resultHit.vector.y = translatedResult.y();
                resultHit.vector.z = translatedResult.z();
            }
        }

        res.hits.push_back(hitDetected);
        res.targets.push_back(resultHit);
    }
    return true;
}

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

    // delete last_octmap_cloud;
    last_octmap_cloud = octTree;
    if (octTree)
    {
        geometry_msgs::TransformStamped transformStamped;
        do
        {
            try
            {
                transformStamped = globalTfBuffer->lookupTransform("odom", "base_footprint", ros::Time(0));
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

        Vector3 position = octomath::Vector3(x, y, z);

        float qx = transformStamped.transform.rotation.x;
        float qy = transformStamped.transform.rotation.y;
        float qz = transformStamped.transform.rotation.z;
        float qu = transformStamped.transform.rotation.w;

        octomath::Quaternion rotation(qu, qx, qy, qz);

        octomath::Vector3 forward = octomath::Vector3(1, 0, 0);
        octomath::Vector3 orientation = rotation.rotate(forward);

        octomath::Vector3 heading(orientation.x(), orientation.y(), 0);

        bool pointsDetected = false;
        sensor_msgs::PointCloud resultCloud;
        resultCloud.header.frame_id = "odom";
        resultCloud.header.stamp = ros::Time();

        vector<float> distances;
        vector<octomath::Vector3 *> hits;
        for (float height = 0.0; height <= 0.4; height += 0.01)
        {
            octomath::Vector3 *result = new octomath::Vector3(0, 0, 0);
            octomath::Vector3 origin(position.x(), position.y(), height);
            if (octTree->castRay(origin, heading, *result, true, 10))
            {
                pointsDetected = true;
                // ROS_INFO("Found thingy!");
                OcTreeNode *hitNode = octTree->search(*result);
                if (hitNode)
                {

                    // std::cout << "Value: " << hitNode->getValue() << "\n";
                    displayMarker(result);
                    geometry_msgs::Point32 hitCoordinates;
                    hitCoordinates.x = result->x();
                    hitCoordinates.y = result->y();
                    hitCoordinates.z = result->z();
                    resultCloud.points.push_back(hitCoordinates);

                    sensor_msgs::ChannelFloat32 channelData;
                    channelData.name = "distance";
                    // channelData.values.push_back(hitNode->getValue());
                    channelData.values.push_back(origin.distanceXY(*result));
                    resultCloud.channels.push_back(channelData);

                    hits.push_back(result);
                    distances.push_back(origin.distanceXY(*result));
                }
                else
                {
                    delete result;
                }
                delete hitNode;
            }
        }

        sensor_msgs::PointCloud obstacleEdgeCloud;
        obstacleEdgeCloud.header.frame_id = "odom";
        obstacleEdgeCloud.header.stamp = ros::Time();

        // obstacle detected
        if (hits.size() > 0)
        {
            float distanceVariation = 0.01;
            float lastObstacledistance = distances[0];
            vector<octomath::Vector3 *> obstacleEdges;
            for (int i = 0; i < hits.size(); i++)
            {
                float height = hits[0]->z();
                float distance = distances[i];
                if (abs(lastObstacledistance - distance) > distanceVariation)
                {
                    if (i > 0)
                    {
                        obstacleEdges.push_back(hits[i - 1]);
                    }
                }
                lastObstacledistance = distance;
                if (i == hits.size() - 1)
                {
                    obstacleEdges.push_back(hits[i]);
                }
            }
            // print data about obstacles
            for (int i = 0; i < obstacleEdges.size(); i++)
            {
                geometry_msgs::Point32 hitCoordinates;
                hitCoordinates.x = obstacleEdges[i]->x();
                hitCoordinates.y = obstacleEdges[i]->y();
                hitCoordinates.z = obstacleEdges[i]->z();
                obstacleEdgeCloud.points.push_back(hitCoordinates);

                sensor_msgs::ChannelFloat32 channelData;
                channelData.name = "height";
                channelData.values.push_back(obstacleEdges[i]->z() - position.z());
                obstacleEdgeCloud.channels.push_back(channelData);

                // cout << "Obstacle height " << obstacleEdges[i]->z() - position.z() << endl;
            }
        }
        // cout << endl;
        // clear memory
        // for (int i = 0; i < hits.size(); i++){
        //     delete hits[i];
        // }
        if (pointsDetected)
        {
            global_collision_pub->publish(resultCloud);
            global_obstacle_edge_pub->publish(obstacleEdgeCloud);
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
        // delete octTree;
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

    ros::Subscriber sub = n.subscribe("/local/octomap_full", 1000, octomapCallback);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("hopper/detected_obstacles/markers", 10);
    ros::Publisher collisionPublisher = n.advertise<sensor_msgs::PointCloud>("hopper/detected_obstacles", 10);
    ros::Publisher obstacleEdges = n.advertise<sensor_msgs::PointCloud>("hopper/detected_obstacles/edges", 10);

    ros::ServiceServer service = n.advertiseService("hopper/detected_obstacles/cast_ray", castRay);

    global_collision_pub = &collisionPublisher;
    global_marker_pub = &marker_pub;
    global_obstacle_edge_pub = &obstacleEdges;

    ros::spin();

    return 0;
}