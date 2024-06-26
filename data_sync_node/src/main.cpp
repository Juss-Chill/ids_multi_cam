#include <data_sync.h>
#include <chrono>
#include <custom_msgs/object_info.h>

// Global variables to control write frequency
std::chrono::steady_clock::time_point last_write_time;
const double write_frequency = 0.5;  // 2 Hz (0.5 seconds interval)
static int frame_cnt = 0;

void bag_write_cb(const sensor_msgs::CompressedImageConstPtr& rcam_img, const sensor_msgs::CompressedImageConstPtr& lcam_img, 
                  const sensor_msgs::PointCloud2ConstPtr vlp_pts, const custom_msgs::object_infoConstPtr& radar_data ,rosbag::Bag& data_bag, std::ofstream& csv_file) {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = now - last_write_time;

    if (elapsed_seconds.count() >= write_frequency) {
        // Reset the last write time
        last_write_time = now;
        frame_cnt++;

        #ifndef WRITE_CSV
        // Write the data to CSV and then write the data to the BAG file
        csv_file << rcam_img->header.stamp << " , " << vlp_pts->header.stamp << " , " << std::abs(rcam_img->header.stamp.toSec() - vlp_pts->header.stamp.toSec()) << std::endl;

        // Write synchronized lidar and camera data to the ROS bag
        data_bag.write("/right_cam/image_rect_color/compressed", rcam_img->header.stamp, *rcam_img);
        data_bag.write("/left_cam/image_rect_color/compressed", lcam_img->header.stamp, *lcam_img);
        data_bag.write("/velodyne_points", vlp_pts->header.stamp, *vlp_pts);
        data_bag.write("/dominant_obj_info", radar_data->header.stamp, *radar_data);
        std::cout << "Frame count : " << frame_cnt << std::endl;
        #endif
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_sync_node");

    // ROS Handles 
    ros::NodeHandle nh;

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
    data_bag.open("fused_r_cam_june26.bag", rosbag::bagmode::Write);
    data_bag.setCompression(rosbag::compression::LZ4);
    #endif

    message_filters::Subscriber<sensor_msgs::CompressedImage> right_cam_img_sub(nh, "/right_cam/image_rect_color/compressed", 10);
    message_filters::Subscriber<sensor_msgs::CompressedImage> left_cam_img_sub(nh, "/left_cam/image_rect_color/compressed", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> vlp_16_points_sub(nh, "/velodyne_points", 10);
    message_filters::Subscriber<custom_msgs::object_info> radar_obj_info_sub(nh, "/dominant_obj_info", 10);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::PointCloud2, custom_msgs::object_info> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), right_cam_img_sub, left_cam_img_sub, vlp_16_points_sub, radar_obj_info_sub);
    sync.registerCallback(boost::bind(&bag_write_cb, _1, _2, _3, _4, boost::ref(data_bag), boost::ref(file)));
    
    // Initialize the last write time
    last_write_time = std::chrono::steady_clock::now();

    ros::spin();

    #ifndef WRITE_CSV
    data_bag.close();
    #endif
}
