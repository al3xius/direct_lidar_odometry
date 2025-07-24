/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"

std::atomic<bool> MapNode::abort_(false);


/**
 * Constructor
 **/

MapNode::MapNode()
: Node("map_node")  // Constructor of the base class rclcpp::Node
{
  this->getParams();

  this->abort_timer = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&MapNode::abortTimerCB, this)
  );

  if (this->publish_full_map_) {
    this->publish_timer = this->create_wall_timer(
      std::chrono::duration<double>(this->publish_freq_),
      std::bind(&MapNode::publishTimerCB, this)
    );
  }

  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "keyframes", rclcpp::QoS(1),
    std::bind(&MapNode::keyframeCB, this, std::placeholders::_1)
  );

  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(1)
  );

  // Example ROS 2 style service creation:
  // this->save_pcd_srv = this->create_service<direct_lidar_odometry::srv::SavePcd>(
  //   "save_pcd", std::bind( &MapNode::savePcd, this, std::placeholders::_1, std::placeholders::_2)
  // );

  this->dlo_map = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

  RCLCPP_INFO(this->get_logger(), "DLO Map Node Initialized");
}


/**
 * Destructor
 **/

MapNode::~MapNode() {}


/**
 * Get Params
 **/

void MapNode::getParams() {

  this->declare_parameter<std::string>("dlo.odomNode.odom_frame", "odom");
  this->get_parameter("dlo.odomNode.odom_frame", this->odom_frame);

  this->declare_parameter<bool>("dlo.mapNode.publishFullMap", true);
  this->get_parameter("dlo.mapNode.publishFullMap", this->publish_full_map_);

  this->declare_parameter<double>("dlo.mapNode.publishFreq", 1.0);
  this->get_parameter("dlo.mapNode.publishFreq", this->publish_freq_);

  this->declare_parameter<double>("dlo.mapNode.leafSize", 0.5);
  this->get_parameter("dlo.mapNode.leafSize", this->leaf_size_);

  // Get Node NS and Remove Leading Character 
  std::string ns = this->get_namespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  //this->odom_frame = ns + "/" + this->odom_frame;

}


/**
 * Start Map Node
 **/

void MapNode::start() {
  RCLCPP_INFO(rclcpp::get_logger("DirectLidarOdometry"), "Starting DLO Map Node");
}


/**
 * Stop Map Node
 **/

void MapNode::stop() {
  RCLCPP_WARN(rclcpp::get_logger("DirectLidarOdometry"), "Stopping DLO Map Node");

  // shutdown
  rclcpp::shutdown();
}


/**
 * Abort Timer Callback
 **/

void MapNode::abortTimerCB() {
  if (abort_) {
    stop();
  }
}


/**
 * Publish Timer Callback
 **/

void MapNode::publishTimerCB() {

  if (this->dlo_map->points.size() == this->dlo_map->width * this->dlo_map->height) {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlo_map, map_ros);
    map_ros.header.stamp = this->get_clock()->now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub->publish(map_ros);

  }
  
}


/**
 * Node Callback
 **/

void MapNode::keyframeCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save keyframe to map
  this->map_stamp = keyframe->header.stamp;
  *this->dlo_map += *keyframe_pcl;

  if (!this->publish_full_map_) {
    if (keyframe_pcl->points.size() == keyframe_pcl->width * keyframe_pcl->height) {
      sensor_msgs::msg::PointCloud2 map_ros;
      pcl::toROSMsg(*keyframe_pcl, map_ros);
      map_ros.header.stamp = map_ros.header.stamp = this->get_clock()->now();;
      map_ros.header.frame_id = this->odom_frame;
      this->map_pub->publish(map_ros);
    }
  }

}

/*
bool MapNode::savePcd(direct_lidar_odometry::save_pcd::Request& req,
                           direct_lidar_odometry::save_pcd::Response& res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlo_map));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlo_map.pcd" << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlo_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res.success;

}*/

