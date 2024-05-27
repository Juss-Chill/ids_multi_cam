/*!
 * \file    acquisitionworker.cpp
 * \author  IDS Imaging Development Systems GmbH
 * \date    2022-06-01
 * \since   1.1.6
 *
 * \brief   The AcquisitionWorker class is used in a worker thread to capture
 *          images from the device continuously and do an image conversion into
 *          a desired pixel format.
 *
 * \version 1.2.1
 *
 * Copyright (C) 2020 - 2023, IDS Imaging Development Systems GmbH.
 *
 * The information in this document is subject to change without notice
 * and should not be construed as a commitment by IDS Imaging Development Systems GmbH.
 * IDS Imaging Development Systems GmbH does not assume any responsibility for any errors
 * that may appear in this document.
 *
 * This document, or source code, is provided solely as an example of how to utilize
 * IDS Imaging Development Systems GmbH software libraries in a sample application.
 * IDS Imaging Development Systems GmbH does not assume any responsibility
 * for the use or reliability of any portion of this document.
 *
 * General permission to copy or modify is hereby granted.
 */

#include "include/acquisitionworker.h"

#include <peak/converters/peak_buffer_converter_ipl.hpp>

#include <QDebug>

#include "include/chronometer.h"
#include "include/mainwindow.h"

#include <QImage>
#include <iostream>

#include <rosbag/bag.h>
#include <sensor_msgs/fill_image.h>


AcquisitionWorker::AcquisitionWorker(MainWindow* parent, DisplayWindow* displayWindow,
    std::shared_ptr<peak::core::DataStream> dataStream, peak::ipl::PixelFormatName pixelFormat,
    size_t imageWidth, size_t imageHeight)
{
    m_parent = parent;
    m_displayWindow = displayWindow;
    m_dataStream = dataStream;
    m_outputPixelFormat = pixelFormat;
    m_imageWidth = imageWidth;
    m_imageHeight = imageHeight;
    m_frameCounter = 0;
    m_errorCounter = 0;

    m_imageConverter = std::make_unique<peak::ipl::ImageConverter>();

    auto isNodeReadable = [this](std::string const& nodeName) {
        if (!m_dataStream->NodeMaps().at(0)->HasNode(nodeName))
        {
            return false;
        }

        return (m_dataStream->NodeMaps().at(0)->FindNode(nodeName)->AccessStatus()
                   == peak::core::nodes::NodeAccessStatus::ReadOnly)
            || (m_dataStream->NodeMaps().at(0)->FindNode(nodeName)->AccessStatus()
                == peak::core::nodes::NodeAccessStatus::ReadWrite);
    };

    m_customNodesAvailable = isNodeReadable("StreamIncompleteFrameCount")
        && isNodeReadable("StreamDroppedFrameCount") && isNodeReadable("StreamLostFrameCount");
}

void AcquisitionWorker::Start()
{
    std::cout << "start Running\n";
    image_transport::ImageTransport it(nh);

    QImage::Format qImageFormat;

    auto bytesPerPixel = size_t{ 0 };

    // camera image publisher
    pub_right_img = it.advertise("/right_cam/image_raw", 100);
    pub_left_img = it.advertise("/left_cam/image_raw", 100);

    // camera_info publisher
    right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/right_cam/camera_info", 100);
    left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/left_cam/camera_info", 100);

#ifndef muni
    auto nodemapRemoteDevice = m_dataStream->ParentDevice()->RemoteDevice()->NodeMaps().at(0);
    // Determine the current DeviceUserID
    std::string device_user_id = nodemapRemoteDevice->FindNode<peak::core::nodes::StringNode>("DeviceUserID")->Value();
    // std::cout << "DeviceUserID :  "<< device_user_id << std::endl;

    if(device_user_id == "right_cam") {
        //std::cout << "Right Image pub success\n";

        // build the camera info for right camera from the parameter server
        nh.getParam("/right_cam/image_width", img_width_right);
        nh.getParam("/right_cam/image_height", img_height_right);
        nh.getParam("/right_cam/distortion_model", distortion_model_right);

        nh.getParam("/right_cam/distortion_coefficients/data", D_right);
        nh.getParam("/right_cam/camera_matrix/data", K_right);
        nh.getParam("/right_cam/rectification_matrix/data", R_right);
        nh.getParam("/right_cam/projection_matrix/data", P_right);

        right_cam_info.height = img_height_right;
        right_cam_info.width = img_width_right;
        right_cam_info.distortion_model = distortion_model_right;

        right_cam_info.D = D_right;

        // K is of type std::vector<double>, converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
        boost::array<double, 9> K_array;
        if (K_right.size() == 9) {
            std::copy(K_right.begin(), K_right.end(), K_array.begin());
        } else {
            // Error handler
            throw std::invalid_argument("K vector must have exactly 9 elements");
        }
        right_cam_info.K = K_array;

        // R is of type std::vector<double> converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
        boost::array<double, 9> R_array;
        if (R_right.size() == 9) {
            std::copy(R_right.begin(), R_right.end(), R_array.begin());
        } else {
            // Error handler
            throw std::invalid_argument("R vector must have exactly 9 elements");
        }
        right_cam_info.R = R_array;

        // P is of type std::vector<double> converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
        boost::array<double, 12> P_array;
        if (P_right.size() == 12) {
            std::copy(P_right.begin(), P_right.end(), P_array.begin());
        } else {
            // Error handler
            throw std::invalid_argument("P vector must have exactly 12 elements");
        }
        right_cam_info.P = P_array;
    }
    else if(device_user_id == "left_cam"){
        //std::cout << "Left Image pub success\n";

        // fill the camera_info data for the left camera
        nh.getParam("/left_cam/image_width", img_width_left);
        nh.getParam("/left_cam/image_height", img_height_left);
        nh.getParam("/left_cam/distortion_model", distortion_model_left);

        nh.getParam("/left_cam/distortion_coefficients/data", D_left);
        nh.getParam("/left_cam/camera_matrix/data", K_left);
        nh.getParam("/left_cam/rectification_matrix/data", R_left);
        nh.getParam("/left_cam/projection_matrix/data", P_left);

        left_cam_info.height = img_height_left;
        left_cam_info.width = img_width_left;
        left_cam_info.distortion_model = distortion_model_left;

        left_cam_info.D = D_left;

        // K is of type std::vector<double>, converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
        boost::array<double, 9> K_array;
        if (K_left.size() == 9) {
            std::copy(K_left.begin(), K_left.end(), K_array.begin());
        } else {
            // Error handler
            throw std::invalid_argument("K vector must have exactly 9 elements");
        }
        left_cam_info.K = K_array;

        // R is of type std::vector<double> converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
        boost::array<double, 9> R_array;
        if (R_left.size() == 9) {
            std::copy(R_left.begin(), R_left.end(), R_array.begin());
        } else {
            // Error handler
            throw std::invalid_argument("R vector must have exactly 9 elements");
        }
        left_cam_info.R = R_array;

        // P is of type std::vector<double> converting the vector data to boost::array<double> for the sensor_msgs/cameraInfo
        boost::array<double, 12> P_array;
        if (P_left.size() == 12) {
            std::copy(P_left.begin(), P_left.end(), P_array.begin());
        } else {
            // Error handler
            throw std::invalid_argument("P vector must have exactly 12 elements");
        }
        left_cam_info.P = P_array;
    }else {
        std::cout << "PUBLISH ERROR, No images are being publushed!!!!\n";
    }
#endif

    // update the server parameters here
    switch (m_outputPixelFormat)
    {
    case peak::ipl::PixelFormatName::BGRa8: 
    {
        qImageFormat = QImage::Format_RGB32;
        bytesPerPixel = 4;
        break;
    }
    case peak::ipl::PixelFormatName::RGB8: 
    {
        qImageFormat = QImage::Format_RGB888;
        bytesPerPixel = 3;
        break;
    }
#if (QT_VERSION >= QT_VERSION_CHECK(5, 5, 0))
    case peak::ipl::PixelFormatName::RGB10p32: {
        qImageFormat = QImage::Format_BGR30;
        bytesPerPixel = 4;
        break;
    }
#endif
    case peak::ipl::PixelFormatName::BayerRG8: 
    case peak::ipl::PixelFormatName::Mono8: 
    {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 5, 0))
        qImageFormat = QImage::Format_Grayscale8;
#else
        qImageFormat = QImage::Format_Indexed8;
#endif
        bytesPerPixel = 1;
        break;
    }
    default: 
    {
        qImageFormat = QImage::Format_RGB32;
        bytesPerPixel = 4;
    }
    }

    // Pre-allocate images for conversion that can be used simultaneously
    // This is not mandatory but it can increase the speed of image conversions
    size_t imageCount = 1;

    // auto nodemapRemoteDevice = m_dataStream->ParentDevice()->RemoteDevice()->NodeMaps().at(0);

    try
    {
        const auto inputPixelFormat = static_cast<peak::ipl::PixelFormatName>(
            nodemapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")
                ->CurrentEntry()
                ->Value());

        m_imageConverter->PreAllocateConversion(
            inputPixelFormat, m_outputPixelFormat, m_imageWidth, m_imageHeight, imageCount);
    }
    catch (const std::exception& e)
    {
        qDebug() << "Exception: " << e.what();
    }

    int incomplete = 0;
    int dropped = 0;
    int lost = 0;

    Chronometer chronometerConversion;
    Chronometer chronometerFrameTime;

    double frameTime_ms = 0;
    double conversionTime_ms = 0;

    m_running = true;

    while (m_running)
    {
        try
        {
            // Wait 5 seconds for an image from the camera
            auto buffer = m_dataStream->WaitForFinishedBuffer(5000);

            chronometerConversion.Start();

            QImage qImage(static_cast<int>(m_imageWidth), static_cast<int>(m_imageHeight), qImageFormat);

            // Create IDS peak IPL image for debayering and convert it to output pixel format

            // Using the image converter ...
            auto image_processed = m_imageConverter->Convert(peak::BufferTo<peak::ipl::Image>(buffer), m_outputPixelFormat,
                qImage.bits(), static_cast<size_t>(qImage.byteCount()));

            // ... or without image converter
            // NOTE: The `ConvertTo` function re-allocates conversion buffers on every call
            //       using the `ImageConverter` should be preferred for performance.
            // peak::BufferTo<peak::ipl::Image>(buffer).ConvertTo(
            //     outputPixelFormat, qImage.bits(), static_cast<size_t>(qImage.byteCount()));

            conversionTime_ms = chronometerConversion.GetTimeSinceStart_ms();
            // Requeue buffer
            m_dataStream->QueueBuffer(buffer);

            // Send signal to update the display
            emit ImageReceived(qImage);
#if 1
                 
            // Publishing raw image
            /*
               a. If there are multiple devices connected, First give a name to them using IDS cockpit or manually in the code
               b. compare the name with the device_user_id publisher and accordingly write a publisher to it and publish the image

                NOTE: In mainwindows.cpp the MAX_NUMBER_OF_DEVICES has been set to 3, please verify why is has been set like that and 
                      modify it accordingly if you are going for more than 3 cameras
                
                NOTE: All the Autoexposure, whitebalance and analoggains has been set to auto in the mainwindow.cpp file
            */
            uint8_t* data_ptr = image_processed.Data();

            std_msgs::Header head_info = std_msgs::Header();
            head_info.stamp = ros::Time::now();
            head_info.frame_id = "sensor_origin";

            sensor_msgs::Image image_msg;
            image_msg.header = head_info;

            sensor_msgs::fillImage( image_msg,
                                    sensor_msgs::image_encodings::BGRA8,
                                    image_processed.Height(),
                                    image_processed.Width(),
                                    image_processed.Width()*4,
                                    data_ptr
                                    );
            if(device_user_id == "right_cam") {
                right_cam_info.header = head_info;
                pub_right_img.publish(image_msg);
                // publishing camera info for the image_proc node
                right_cam_info_pub.publish(right_cam_info);
            }
            else if(device_user_id == "left_cam"){
                left_cam_info.header = head_info;
                pub_left_img.publish(image_msg);
                // publishing camera info for the image_proc node
                left_cam_info_pub.publish(left_cam_info);
            }else {
                std::cout << "PUBLISH ERROR, No images are being publushed!!!!\n";
            }
#endif
            frameTime_ms = chronometerFrameTime.GetTimeSinceStart_ms();
            chronometerFrameTime.Start();

            m_frameCounter++;

            if (m_customNodesAvailable)
            {
                // Missing packets on the interface, event after 1 resend
                incomplete = static_cast<int>(
                    m_dataStream->NodeMaps()
                        .at(0)
                        ->FindNode<peak::core::nodes::IntegerNode>("StreamIncompleteFrameCount")
                        ->Value());

                // Camera buffer overrun (sensor data too fast for interface)
                dropped = static_cast<int>(
                    m_dataStream->NodeMaps()
                        .at(0)
                        ->FindNode<peak::core::nodes::IntegerNode>("StreamDroppedFrameCount")
                        ->Value());

                // User buffer overrun (application too slow to process the camera data)
                lost = static_cast<int>(m_dataStream->NodeMaps()
                                            .at(0)
                                            ->FindNode<peak::core::nodes::IntegerNode>("StreamLostFrameCount")
                                            ->Value());
            }
        }
        catch (const std::exception&)
        {
            // Without a sleep the GUI will be blocked completely and the CPU load will rise!
            QThread::msleep(1000);

            m_errorCounter++;
        }

        emit UpdateCounters(frameTime_ms, conversionTime_ms, m_frameCounter, m_errorCounter, incomplete, dropped, lost,
            m_customNodesAvailable);
    }
}

void AcquisitionWorker::Stop()
{
    m_running = false;
}
