% Input: ROSBAG with the two camera(compressed) images and one pointcloud data
% Output: Images are extracted into .png into respective folders and pointcloud is extracted into the .pcd format.
% The Input and output is chosen to be aligned with the lidar-camera calibration application from MATLAB
% Note: Its better to provide the synchornized data in the ROSBAG itself eventhough this script explicity does the sync
%

clc;
clear all;
close all;

path = "./fused_lidar_cam.bag";

bag = rosbag(path);

% create Objects
right_img_bag = select(bag, "Topic","/right_cam/image_rect_color/compressed");
left_img_bag = select(bag, "Topic","/left_cam/image_rect_color/compressed");
pc_bag = select(bag, "Topic","/merged_cloud");

%Read the messages of respective Topics
right_img_msgs = readMessages(right_img_bag);
left_img_msgs = readMessages(left_img_bag);
pc_msgs = readMessages(pc_bag);

% prepare the timeseries objects
ts1 = timeseries(right_img_bag);
ts2 = timeseries(pc_bag);
ts3 = timeseries(left_img_bag);

t1 = ts1.Time;
t2 = ts2.Time;
t3 = ts3.Time;

disp("Created Time series")
k = 1;
% Filtering again to make sure the pointCloud data and the image data are
% with in 0.1sec Tolerance
% 
if size(t2,1) > size(t1,1) % mode point cloud data compared to image data
    for i = 1:size(t1,1) % capping to the size of the least object's data points
        [val, indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx(k, :) = [i indx];
            k = k+1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end


pcFilesPath = fullfile("./extracted_pcd");
imageFilesPath = fullfile("./extracted_right_img");
imageFilesPath_2 = fullfile("./extracted_left_img");

if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if ~exist(imageFilesPath_2,'dir')
    mkdir(imageFilesPath_2);
end
if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end

disp("Geenrating images and PCD files")
for i = 1:length(idx)
    I = readImage(right_img_msgs{idx(i,1)});
    I_2 = readImage(left_img_msgs{idx(i,1)});
    pc = pointCloud(readXYZ(pc_msgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imageFileName_2 = strcat(imageFilesPath_2,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    imwrite(I_2, imageFileName_2)
    pcwrite(pc,pcFileName);
end
disp("Done Processing, Please launch the calibration application for calibration")