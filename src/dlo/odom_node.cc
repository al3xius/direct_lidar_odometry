/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/odom.h"

void controlC(int sig) {

  dlo::OdomNode::abort();

}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dlo_odom_node");
  rclcpp::Node nh("~");

  signal(SIGTERM, controlC);
  sleep(0.5);

  dlo::OdomNode node(nh);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
