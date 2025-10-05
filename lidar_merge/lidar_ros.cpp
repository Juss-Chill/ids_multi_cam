#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// Eigen Lib
#include <Eigen/Dense>

// TF2 headers
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

/*
Description:
This node listens to the pointcloud from the left lidar, right lidars and listens dynamic transformation from the Autoware node (Transformation between the left and right lidar is computed using NDT transform)
and tranforms the left pointcloud to the right lidar pointcloud frame

Inputs:
a. Left lidar pointcloud (source frame)
b. Right lidar pointcloud (Target frame)
c. Trasform that moves the Left lidar PC to right lidar PC frame.

Output:
Merged pointlcoud from the left and right lidar
*/

class DualLidarFusion {
    ros::NodeHandle nh_, pnh_;
    ros::Publisher merged_pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_right_, sub_left_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2> SyncPolicy;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();
    std::string frame_id_;

    // listen to the Autoware published transform and store it in buffer
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string parent_frame_;  // e.g. right lidar
    std::string child_frame_;   // e.g. left lidar
    std::string output_frame_;  // output merged cloud frame

public:
    DualLidarFusion()
        : nh_(), pnh_("~"),
          sub_right_(nh_, pnh_.param<std::string>("right_topic", "/ouster1/points"), 10),
          sub_left_(nh_, pnh_.param<std::string>("left_topic", "/ouster2/points"), 10),
          tf_listener_(tf_buffer_)
    {
        parent_frame_ = pnh_.param<std::string>("parent_frame", "os_sensor_right");
        child_frame_  = pnh_.param<std::string>("child_frame", "os_sensor_left");
        output_frame_ = pnh_.param<std::string>("output_frame", "os_sensor_right");

        #if 0
        // --- Transformation params ---
        std::vector<double> tr_vals{0.00923139, 1.02588, 0.00465358, 0.0174707, 0.000347, -0.0116989, -0.9997789}; // x, y, z, qx, qy, qz, qw //  0, 1.005, 0, 0, 0, 0, 1
        pnh_.param<std::vector<double>>("transformation_params", tr_vals, tr_vals);

        Eigen::Quaternionf q(tr_vals[6], tr_vals[3], tr_vals[4], tr_vals[5]);
        q.normalize();
        transform_.translation() << tr_vals[0], tr_vals[1], tr_vals[2];
        transform_.rotate(q);
        #endif

        // Output
        frame_id_ = pnh_.param<std::string>("output_frame", "os_sensor_right");
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_cloud", 10);

        // Synchronizer
        SyncPolicy policy(10);                                    // queue size
        policy.setMaxIntervalDuration(ros::Duration(0.1));        // allow from 100ms skew
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(policy));
        sync_->connectInput(sub_right_, sub_left_);
        sync_->registerCallback(boost::bind(&DualLidarFusion::callback, this, _1, _2));
    }
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& r,
                  const sensor_msgs::PointCloud2ConstPtr& l) 
    {
        std::cout << "Sync success\n";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clt(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*r, *cr);
        pcl::fromROSMsg(*l, *cl);

        // Look for the transformation in the buffer
        geometry_msgs::TransformStamped tf_msg = 
                tf_buffer_.lookupTransform(parent_frame_,   // target (right lidar)
                                           child_frame_,    // source (left lidar)
                                           r->header.stamp,
                                           ros::Duration(0.1)); // fail check for 0.1s

        Eigen::Affine3d T = tf2::transformToEigen(tf_msg.transform);
        Eigen::Affine3f T_float = T.cast<float>();

        // Transform left lidar PC into right lidar frame
        // pcl::transformPointCloud(*cl, *clt, transform_);
        pcl::transformPointCloud(*cl, *clt, T_float);

        // Merge
        *cr += *clt;

        // Publish
        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(*cr, out);
        out.header.stamp = r->header.stamp;   // keep lidar timestamp
        out.header.frame_id = frame_id_;
        merged_pub_.publish(out);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_lidar_fusion");
    ROS_INFO("dual_lidar_fusion node started");
    DualLidarFusion node;
    ros::spin();
    return 0;
}
