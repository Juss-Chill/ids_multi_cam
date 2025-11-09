#include <data_sync.h>
#include <chrono>

#include<custom_msgs/RadarDetection.h>
#include<custom_msgs/RadarDetectionArray.h>
#include<custom_msgs/imu_data.h>

#include <tf2_msgs/TFMessage.h>
#include <ros/ros.h>

/*
Task Intended: MATLAB Lidar-Camera registration

The below script scynchrinizes the data from the two cameras(left and right) and lidar(merged pointcloud from the right and left lidar),
synchronizes them and stores them in the ROSBAG

Inputs:
a. Left camera images(compressed format) - dependency on ROS ImageProc plugin
b. Right camera images(compressed format)- dependency on ROS ImageProc plugin
c. Lidar pointlcoud (If two lidars are used, make sure to transform the rest of the lidars to the single lidar frame)

output:
ROS bag with poitclouds(all provided[Individual PCs + merged PC]), camera images

Note: If needed log the TF's seperately into the text file, although the modules are present here, not used for the curent application
*/


// Global variables to control write frequency
std::chrono::steady_clock::time_point last_write_time;
double WRITE_FREQUENCY = 0.5;  // 2 Hz (0.5 seconds interval)
std::string BAG_NAME = "all_sensor_data.bag";
static int frame_cnt = 0;

void bag_write_cb(const sensor_msgs::CompressedImageConstPtr& rcam_img, const sensor_msgs::CompressedImageConstPtr& lcam_img, 
                  const sensor_msgs::PointCloud2ConstPtr left_lidar_pts, const sensor_msgs::PointCloud2ConstPtr right_lidar_pts,
                  const sensor_msgs::PointCloud2ConstPtr merged_lidar_pts,
                  const custom_msgs::RadarDetectionArrayConstPtr& radar_detections,
                  const custom_msgs::imu_dataConstPtr& imu_gps,
                  rosbag::Bag& data_bag) {

            //             ROS_INFO_STREAM("Right cam: " << rcam_img->header.stamp.toSec()
            // << ", Left cam: " << lcam_img->header.stamp.toSec()
            // << ", Left lidar: " << left_lidar_pts->header.stamp.toSec()
            // << ", Right lidar: " << right_lidar_pts->header.stamp.toSec()
            // << ", Merged lidar: " << merged_lidar_pts->header.stamp.toSec());

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - last_write_time;

    if (elapsed_seconds.count() >= WRITE_FREQUENCY) {
        // Reset the last write time
        last_write_time = now;
        frame_cnt++;

        #ifndef WRITE_CSV
        // Write the data to CSV and then write the data to the BAG file
        // csv_file << rcam_img->header.stamp << " , " << vlp_pts->header.stamp << " , " << std::abs(rcam_img->header.stamp.toSec() - vlp_pts->header.stamp.toSec()) << std::endl;

        // Write synchronized lidar and camera data to the ROS bag
        data_bag.write("/right_cam/image_rect_color/compressed", rcam_img->header.stamp, *rcam_img);
        data_bag.write("/left_cam/image_rect_color/compressed", lcam_img->header.stamp, *lcam_img);
        data_bag.write("/ouster2/points", left_lidar_pts->header.stamp, *left_lidar_pts);
        data_bag.write("/ouster1/points", right_lidar_pts->header.stamp, *right_lidar_pts);
        data_bag.write("/merged_cloud", merged_lidar_pts->header.stamp, *merged_lidar_pts);
        data_bag.write("/radar_detections", radar_detections->header.stamp, *radar_detections);
        data_bag.write("/imu_gps_synced_data", imu_gps->header.stamp, *imu_gps);

        std::cout << "Frame count : " << frame_cnt << std::endl;
        #endif
    }
}

// Store the transforms published by the Autoware module
void tf_cb(const tf2_msgs::TFMessage::ConstPtr& msg, rosbag::Bag& data_bag) {
    if (!msg->transforms.empty()) {
        data_bag.write("/tf", msg->transforms[0].header.stamp, *msg);
    }
}

// Store the static transform between the lidar-sensor frames provided by the OUSTER ROS module
void tf_static_cb(const tf2_msgs::TFMessage::ConstPtr& msg, rosbag::Bag& data_bag) {
    if (!msg->transforms.empty()) {
        data_bag.write("/tf_static", msg->transforms[0].header.stamp, *msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_sync_node");

    // ROS Handles 
    ros::NodeHandle nh("~");
    nh.param<double>("write_frequency", WRITE_FREQUENCY, 0.5); // data will be recorded for every 0.5 seconds
    nh.param<std::string>("bag_name", BAG_NAME, "all_sensor_data.bag");

    std::cout << "*********Initialised data sync node*********" << std::endl;

    #ifndef WRITE_CSV // Create a file and write the lidar and camera data to the file
    std::string f_name = "rcam_lidar_debug.csv";

    std::ofstream file(f_name);

    if (!file.good()) {
        // Fopen error
        std::ofstream createFile(f_name); // Creates new file
        createFile << "R_cam,VLP16,Diff\n"; // Header info
        // createFile.close();
    }

    rosbag::Bag data_bag;
    data_bag.open(BAG_NAME, rosbag::bagmode::Write);
    data_bag.setCompression(rosbag::compression::LZ4);
    #endif

    // camera images
    message_filters::Subscriber<sensor_msgs::CompressedImage> right_cam_img_sub(nh, "/right_cam/image_rect_color/compressed", 50);
    message_filters::Subscriber<sensor_msgs::CompressedImage> left_cam_img_sub(nh, "/left_cam/image_rect_color/compressed", 50);
    
    // LiDAR pointcloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> left_lidar_sub(nh, "/ouster2/points", 50);
    message_filters::Subscriber<sensor_msgs::PointCloud2> right_lidar_sub(nh, "/ouster1/points", 50);
    message_filters::Subscriber<sensor_msgs::PointCloud2> merged_lidar_sub(nh, "/merged_cloud", 50);

    // Radar data
    message_filters::Subscriber<custom_msgs::RadarDetectionArray> radar_detections_sub(nh, "/radar_detections", 50);

    // IMU+GPS data
    message_filters::Subscriber<custom_msgs::imu_data> imu_gps_sub(nh, "/imu_gps_synced_data", 50);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, 
                                                            sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                            custom_msgs::RadarDetectionArray
                                                            ,custom_msgs::imu_data
                                                            > MySyncPolicy;


    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), right_cam_img_sub, left_cam_img_sub, left_lidar_sub, right_lidar_sub, 
                                                                                  merged_lidar_sub, radar_detections_sub, imu_gps_sub)); // , imu_gps_sub
    sync_->registerCallback(boost::bind(&bag_write_cb, _1, _2, _3, _4, _5, _6, _7, boost::ref(data_bag)));


    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), right_cam_img_sub, left_cam_img_sub, left_lidar_sub, right_lidar_sub, merged_lidar_sub);
    // sync.registerCallback(boost::bind(&bag_write_cb, _1, _2, _3, _4, _5, boost::ref(data_bag)));

    // ros::Subscriber tf_sub = nh.subscribe<tf2_msgs::TFMessage>(
    //     "/tf", 50, boost::bind(&tf_cb, _1, boost::ref(data_bag)));

    // ros::Subscriber tf_static_sub = nh.subscribe<tf2_msgs::TFMessage>(
    //     "/tf_static", 10, boost::bind(&tf_static_cb, _1, boost::ref(data_bag)));

    
    // Initialize the last write time
    last_write_time = std::chrono::steady_clock::now();

    ros::spin();

    #ifndef WRITE_CSV
    data_bag.close();
    #endif
}
