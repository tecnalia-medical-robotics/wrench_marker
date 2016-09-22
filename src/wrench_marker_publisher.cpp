/**
 * @file   WrenchMarkerPublisher.cpp
 * @author Miguel Prada <miguel.prada@tecnalia.com>
 * @date   2016
 *
 * Copyright 2016 Tecnalia Research & Innovation.
 * Distributed under the GNU GPL v3. For full terms see https://www.gnu.org/licenses/gpl.txt
 *
 * @brief  Node which listens to geometry_msgs/Wrench messages and publishes markers to visualize in rviz.
 */

#include <wrench_marker/wrench_marker.h>

#include <geometry_msgs/Wrench.h>

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<wrench_marker::WrenchMarker> WrenchMarkerPtr;

class WrenchMarkerPublisher
{

public:

  WrenchMarkerPublisher()
  {

    ros::NodeHandle p_nh("~");
    if( !p_nh.getParam( "wrench_marker_frame", _wrench_marker_frame ) )
    {
      ROS_WARN( "Could not read 'wrench_marker_frame' parameter. Using 'world' as reference for the marker" );
      _wrench_marker_frame = "world";
    }

    _wrench_marker.reset( new wrench_marker::WrenchMarker( _wrench_marker_frame ) );

    _wrench_sub = _nh.subscribe( "wrench", 1, &WrenchMarkerPublisher::wrench_cb, this );
    _wrench_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>( "wrench_marker", 1 );

  }

private:

  void wrench_cb( const geometry_msgs::WrenchConstPtr& wrench_msg )
  {

    const visualization_msgs::MarkerArray& wrench_marker_msg = _wrench_marker->populate( *wrench_msg );
    _wrench_marker_pub.publish( wrench_marker_msg );

  }

  ros::NodeHandle _nh;

  std::string _wrench_marker_frame;

  WrenchMarkerPtr _wrench_marker;

  ros::Subscriber _wrench_sub;
  ros::Publisher _wrench_marker_pub;

};

int main( int argc, char** argv )
{

  ros::init( argc, argv, "wrench_marker_publisher" );
  ros::AsyncSpinner spinner(1);

  WrenchMarkerPublisher wrench_marker_publisher;

  spinner.start();
  ros::waitForShutdown();

  return 0;

}
