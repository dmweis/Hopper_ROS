#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "octomap/AbstractOcTree.h"
#include "octomap/math/Vector3.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace octomap;


tf2_ros::Buffer *globalTfBuffer;
tf2_ros::TransformListener *globalTfListener;

void octomapCallback(const octomap_msgs::Octomap &msg)
{
    // ROS_INFO("Hi [%s]", msg.header.frame_id.c_str());
    octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(msg);
    octomap::OcTree *octTree = dynamic_cast<octomap::OcTree *>(tree);
    if (octTree)
    {
        geometry_msgs::TransformStamped transformStamped;

        while (true)
        {
            try
            {
                transformStamped = globalTfBuffer->lookupTransform("odom", "body_link",
                                                                   ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }
        float x = transformStamped.transform.translation.x;
        float y = transformStamped.transform.translation.y;
        float z = transformStamped.transform.translation.z;
        octomath::Vector3 a = octomath::Vector3(x, y, z);

        float qx = transformStamped.transform.rotation.x;
        float qy = transformStamped.transform.rotation.y;
        float qz = transformStamped.transform.rotation.z;
        float qu = transformStamped.transform.rotation.w;
        octomath::Quaternion rotation(qu, qx, qy, qz);

        octomath::Vector3 front = octomath::Vector3(1, 0, 0);
        octomath::Vector3 b = rotation.rotate(front);
        octomath::Vector3 heading(b.x(), b.y(), 0.0);
        octomath::Vector3 result = octomath::Vector3(0, 0, 0);
        std::cout << "origin " << a << std::endl;
        std::cout << "direction " << heading << std::endl;
        std::cout << "Result " << result << std::endl;
        if (octTree->castRay(a, heading, result, true, 10))
        {
            ROS_INFO("Found thingy!");
            OcTreeNode *n = octTree->search(result);
            if (n)
            {
                std::cout << "Value: " << n->getValue() << "\n";
            }
        }
        else
        {
            // ROS_INFO("Not found thingy");
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
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    globalTfBuffer = &tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    globalTfListener = &tfListener;

    ros::Subscriber sub = n.subscribe("/global/octomap_full", 1000, octomapCallback);

    ros::spin();

    return 0;
}