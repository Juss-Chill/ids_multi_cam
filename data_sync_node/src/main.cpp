// #include<data_sync_node/data_sync.h>
#include<data_sync.h>

void bag_write_cb(const sensor_msgs::ImageConstPtr& rcam_img, const sensor_msgs::PointCloud2ConstPtr vlp_pts, rosbag::Bag& data_bag, std::ofstream& csv_file) {
//    std::cout << "Data recived..printing Time stamps >>>>>\n\n" << std::endl;
//    std::cout << "RCAM : " << rcam_img->header.stamp << "\nVLP : "<< vlp_pts->header.stamp << 
//             "\nDiff : " << std::abs(rcam_img->header.stamp.toSec() - vlp_pts->header.stamp.toSec()) << std::endl;

#ifndef WRITE_CSV
    // write the data to CSV and then write the data to the BAG file
    // std::string f_name = "rcam_lidar_debug.csv";
    // std::ofstream outFile;

    // outFile.open(f_name, std::ios::app);

    // // Check if the file is opened successfully
    // if (!outFile.is_open()) {
    //     std::cout << "Error opening file for writing!";
    //     exit(-1); // Exit the program with an error code
    // }
    // outFile << "R_cam,VLP16,Diff\n"; //Header info
    csv_file << rcam_img->header.stamp << " , " << vlp_pts->header.stamp << " , " << std::abs(rcam_img->header.stamp.toSec() - vlp_pts->header.stamp.toSec()) << std::endl;
    // outFile.close();
    // std::cout << "File Write Success, closed\n";

    // Write synchronized lidar and camera data to the ROS bag
    data_bag.write("/right_cam/image_rect_color", rcam_img->header.stamp, *rcam_img);
    data_bag.write("/velodyne_points", vlp_pts->header.stamp, *vlp_pts);
#endif
    // rosbag::Bag camera_bag;
    // camera_bag.open("/home/shreyash/workspace/ids_ros_ws/src/fused_r_cam.bag", rosbag::bagmode::Append);
    // camera_bag.setCompression(rosbag::compression::LZ4);
    // camera_bag.write("/right_cam/image_rect_color", rcam_img->header.stamp, *rcam_img);


}

int main(int argc, char **argv) {
    ros::init(argc, argv, "data_sync_node");

    // ROS Handles 
    ros::NodeHandle nh;

    std::cout << "HELLO NEW NODE\n\n\n" << std::endl;



#ifndef WRITE_CSV // Create a file and write the lidar and camera data to the file
    std::string f_name = "rcam_lidar_debug.csv";
    std::ofstream file(f_name);

    if (!file.good()) {
        //Fopen error
        std::ofstream createFile(f_name); //creates new file
        createFile << "R_cam,VLP16,Diff\n"; //Header info
        // createFile.close();
    }

    rosbag::Bag data_bag;
    data_bag.open("fused_r_cam.bag", rosbag::bagmode::Write);
    data_bag.setCompression(rosbag::compression::LZ4);
#endif

    message_filters::Subscriber<sensor_msgs::Image> right_cam_img_sub(nh, "/right_cam/image_rect_color",10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> vlp_16_points_sub(nh, "/velodyne_points",10);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), right_cam_img_sub, vlp_16_points_sub);
    sync.registerCallback(boost::bind(&bag_write_cb, _1, _2, boost::ref(data_bag), boost::ref(file)));
    ros::spin();

#ifndef WRITE_CSV
    data_bag.close();
#endif
}