/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

// src/dlo/map_main.cc

#include "dlo/map.h"
#include "rclcpp/rclcpp.hpp"
#include <signal.h>
#include <unistd.h>

void controlC(int sig) {
  MapNode::abort();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGTERM, controlC);
  sleep(0.5);

  auto node = std::make_shared<MapNode>();
  node->start();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

