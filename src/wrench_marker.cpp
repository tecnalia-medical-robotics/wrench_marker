/**
 * @file   wrench_marker.cpp
 * @author Miguel Prada <miguel.prada@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief  Helper class to produce visualization_msgs::MarkerArray messages from a geometry_msgs::Wrench message.
 */

#include <wrench_marker/wrench_marker.h>

namespace wrench_marker
{

WrenchMarker::WrenchMarker( const std::string& frame, const std::string& ns, visualization_msgs::MarkerArray* msg )
  : msg_( msg )
{

  if( msg_ )
  {
    msg_->markers.clear();
  }
  else
  {
    msg_ = new visualization_msgs::MarkerArray;
  }

  visualization_msgs::Marker aux_marker;
  aux_marker.header.frame_id = frame;
  aux_marker.ns = ns;
  aux_marker.id = 0;
  aux_marker.type = visualization_msgs::Marker::ARROW;
  aux_marker.action = visualization_msgs::Marker::ADD;
  aux_marker.scale.x = ARROW_SHAFT_DIAMETER;
  aux_marker.scale.y = ARROW_HEAD_DIAMETER;
  aux_marker.color.r = 1.0;
  aux_marker.color.g = 0.0;
  aux_marker.color.b = 1.0;
  aux_marker.color.a = 1.0;
  aux_marker.lifetime = ros::Duration( 0.1 );
  aux_marker.points.resize( 2 );
  msg_->markers.push_back( aux_marker );

  // Markers for torque in X axis
  aux_marker.color.b = 0.0;
  
  aux_marker.id = 1;
  msg_->markers.push_back( aux_marker );
  aux_marker.id = 2;
  msg_->markers.push_back( aux_marker );

  // Markers for torque in Y axis
  aux_marker.color.r = 0.0;
  aux_marker.color.g = 1.0;

  aux_marker.id = 3;
  msg_->markers.push_back( aux_marker );
  aux_marker.id = 4;
  msg_->markers.push_back( aux_marker );

  // Markers for torque in Z axis
  aux_marker.color.g = 0.0;
  aux_marker.color.b = 1.0;

  aux_marker.id = 5;
  msg_->markers.push_back( aux_marker );
  aux_marker.id = 6;
  msg_->markers.push_back( aux_marker );

}

const visualization_msgs::MarkerArray& WrenchMarker::populate( const geometry_msgs::Wrench& wrench, const ros::Time& time, const geometry_msgs::Point& pos )
{

  msg_->markers[0].header.stamp = time;
  msg_->markers[0].points[0].x = pos.x;
  msg_->markers[0].points[0].y = pos.y;
  msg_->markers[0].points[0].z = pos.z;
  msg_->markers[0].points[1].x = pos.x + FORCE_ARROW_SCALE * wrench.force.x;
  msg_->markers[0].points[1].y = pos.y + FORCE_ARROW_SCALE * wrench.force.y;
  msg_->markers[0].points[1].z = pos.z + FORCE_ARROW_SCALE * wrench.force.z;

  msg_->markers[1].header.stamp = time;
  msg_->markers[1].points[0].x = pos.x;
  msg_->markers[1].points[0].y = pos.y + TORQUE_ARROW_SEPARATION/2;
  msg_->markers[1].points[0].z = pos.z - TORQUE_ARROW_SCALE * wrench.torque.x;
  msg_->markers[1].points[1].x = pos.x;
  msg_->markers[1].points[1].y = pos.y + TORQUE_ARROW_SEPARATION/2;
  msg_->markers[1].points[1].z = pos.z + TORQUE_ARROW_SCALE * wrench.torque.x;

  msg_->markers[2].header.stamp = time;
  msg_->markers[2].points[0].x = pos.x;
  msg_->markers[2].points[0].y = pos.y - TORQUE_ARROW_SEPARATION/2;
  msg_->markers[2].points[0].z = pos.z + TORQUE_ARROW_SCALE * wrench.torque.x;
  msg_->markers[2].points[1].x = pos.x;
  msg_->markers[2].points[1].y = pos.y - TORQUE_ARROW_SEPARATION/2;
  msg_->markers[2].points[1].z = pos.z - TORQUE_ARROW_SCALE * wrench.torque.x;

  msg_->markers[3].header.stamp = time;
  msg_->markers[3].points[0].x = pos.x - TORQUE_ARROW_SCALE * wrench.torque.y;
  msg_->markers[3].points[0].y = pos.y;
  msg_->markers[3].points[0].z = pos.z + TORQUE_ARROW_SEPARATION/2;
  msg_->markers[3].points[1].x = pos.x + TORQUE_ARROW_SCALE * wrench.torque.y;
  msg_->markers[3].points[1].y = pos.y;
  msg_->markers[3].points[1].z = pos.z + TORQUE_ARROW_SEPARATION/2;

  msg_->markers[4].header.stamp = time;
  msg_->markers[4].points[0].x = pos.x + TORQUE_ARROW_SCALE * wrench.torque.y;
  msg_->markers[4].points[0].y = pos.y;
  msg_->markers[4].points[0].z = pos.z - TORQUE_ARROW_SEPARATION/2;
  msg_->markers[4].points[1].x = pos.x - TORQUE_ARROW_SCALE * wrench.torque.y;
  msg_->markers[4].points[1].y = pos.y;
  msg_->markers[4].points[1].z = pos.z - TORQUE_ARROW_SEPARATION/2;

  msg_->markers[5].header.stamp = time;
  msg_->markers[5].points[0].x = pos.x + TORQUE_ARROW_SEPARATION/2;
  msg_->markers[5].points[0].y = pos.y - TORQUE_ARROW_SCALE * wrench.torque.z;
  msg_->markers[5].points[0].z = pos.z;
  msg_->markers[5].points[1].x = pos.x + TORQUE_ARROW_SEPARATION/2;
  msg_->markers[5].points[1].y = pos.y + TORQUE_ARROW_SCALE * wrench.torque.z;
  msg_->markers[5].points[1].z = pos.z;

  msg_->markers[6].header.stamp = time;
  msg_->markers[6].points[0].x = pos.x - TORQUE_ARROW_SEPARATION/2;
  msg_->markers[6].points[0].y = pos.y + TORQUE_ARROW_SCALE * wrench.torque.z;
  msg_->markers[6].points[0].z = pos.z;
  msg_->markers[6].points[1].x = pos.x - TORQUE_ARROW_SEPARATION/2;
  msg_->markers[6].points[1].y = pos.y - TORQUE_ARROW_SCALE * wrench.torque.z;
  msg_->markers[6].points[1].z = pos.z;

  return *msg_;

}

}
