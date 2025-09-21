#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

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

public:
    DualLidarFusion()
        : nh_(), pnh_("~"),
          sub_right_(nh_, pnh_.param<std::string>("right_topic", "/ouster1/points"), 10),
          sub_left_(nh_, pnh_.param<std::string>("left_topic", "/ouster2/points"), 10)
    {
        // --- Transformation params ---
        std::vector<double> tr_vals{0, 1.005, 0, 0, 0, 0, 1}; // x, y, z, qx, qy, qz, qw
        pnh_.param<std::vector<double>>("transformation_params", tr_vals, tr_vals);

        Eigen::Quaternionf q(tr_vals[6], tr_vals[3], tr_vals[4], tr_vals[5]);
        q.normalize();
        transform_.translation() << tr_vals[0], tr_vals[1], tr_vals[2];
        transform_.rotate(q);

        // --- Output setup ---
        frame_id_ = pnh_.param<std::string>("output_frame", "os_sensor_right");
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_cloud", 10);

        // --- Synchronizer setup ---
        SyncPolicy policy(10);                                    // queue size
        policy.setMaxIntervalDuration(ros::Duration(0.02));       // allow up to 20 ms skew
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(policy));
        sync_->connectInput(sub_right_, sub_left_);
        sync_->registerCallback(boost::bind(&DualLidarFusion::callback, this, _1, _2));
    }
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& r,
                  const sensor_msgs::PointCloud2ConstPtr& l) 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clt(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*r, *cr);
        pcl::fromROSMsg(*l, *cl);

        // Transform left lidar into right lidar frame
        pcl::transformPointCloud(*cl, *clt, transform_);

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
