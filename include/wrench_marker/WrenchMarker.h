///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2016 Tecnalia
//
// http://www.tecnalia.com/
//
// Health Division - Medical Robotics Group
// http://www.tecnalia.com/en/health/index.htm
//
// If you find any bug or if you have any question please contact
// Miguel Prada <miguel.prada@tecnalia.com>
//
// Brief :
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef WRENCH_MARKER_WRENCH_MARKER_H
#define WRENCH_MARKER_WRENCH_MARKER_H

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Wrench.h>
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

  WrenchMarker( const std::string& frame, const std::string& ns = "wrench", visualization_msgs::MarkerArray* msg = 0 )
    : msg_( msg )
  {

    if( !msg_ )
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

  const visualization_msgs::MarkerArray& populate( const geometry_msgs::Wrench& wrench, const ros::Time& time = ros::Time::now(), const geometry_msgs::Point& pos = geometry_msgs::Point() )
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

private:

  visualization_msgs::MarkerArray* msg_;

};

}

#endif
