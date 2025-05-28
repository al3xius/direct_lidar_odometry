/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/
#pragma once


#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/times.h>
#include <thread>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nano_gicp/nano_gicp.hpp>
typedef pcl::PointXYZI PointType;

#include <chrono>
#include <iostream>

class Timer {
public:

    Timer() { tic(); }
    Timer(const std::string& nameIn) : name(nameIn) { tic(); }


    void tic() { start = std::chrono::system_clock::now(); }


    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = end - start;
        start = end;
        return dt.count() * 1000;
    }

    void toc_cout() { std::cout << "[" << name << "]:" << toc() << "ms" << std::endl; }

private:
    const std::string name;
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#define TIMER_CREATE(name) Timer name(#name)