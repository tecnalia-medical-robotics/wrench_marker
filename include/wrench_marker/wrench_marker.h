/**
 * @file   wrench_marker.h
 * @author Miguel Prada <miguel.prada@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief  Helper class to produce visualization_msgs::MarkerArray messages from a geometry_msgs::Wrench message.
 */

#ifndef WRENCH_MARKER_WRENCH_MARKER_H
#define WRENCH_MARKER_WRENCH_MARKER_H

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>

namespace wrench_marker
{

const double ARROW_SHAFT_DIAMETER = 0.01;
const double ARROW_HEAD_DIAMETER = 0.02;

const double FORCE_ARROW_SCALE = 0.1;

const double TORQUE_ARROW_SCALE = 0.1;
const double TORQUE_ARROW_SEPARATION = 0.1;

class WrenchMarker
{

public:

  WrenchMarker( const std::string& ns = "wrench", visualization_msgs::MarkerArray* msg = 0 );

  const visualization_msgs::MarkerArray& getMarker( const geometry_msgs::WrenchStamped& wrench_stamped, const ros::Time& time = ros::Time::now() );

  const visualization_msgs::MarkerArray& getMarker( const geometry_msgs::Wrench& wrench, const std::string& frame, const geometry_msgs::Point& pos = geometry_msgs::Point(), const ros::Time& time = ros::Time::now() );

private:

  visualization_msgs::MarkerArray* msg_;

};

}

#endif
