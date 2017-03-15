# Wrench Marker

[![Build status badge](https://travis-ci.org/tecnalia-medical-robotics/wrench_marker.svg?branch=indigo-devel)](https://travis-ci.org/tecnalia-medical-robotics/wrench_marker)

This package contains both a library and an executable node.

The `wrench_marker_publisher` node can be used to generate marker messages out of wrenches published to a ROS topic.

It internally uses the `wrench_marker` library.
This library exports the `wrench_marker/wrench_marker.h` header, in which a single class is declared.
It can be used to generate rviz markers (i.e. `visualization_msgs/MarkerArray` messages) from wrenches (i.e. `geometry_msgs/Wrench` messages) that use several arrow markers to display the wrench in rviz.
The responsibility of publishing the message lies in user code.

![Wrench marker on LWR](http://i.imgur.com/9a1wh65.png)

## `wrench_marker_publisher` node

The `wrench_marker_publisher` node subscribes to `geometry_msgs/WrenchStamped` messages in the `wrench` topic, and publishes a marker array to visualize it in `wrench_marker` topic.

The generated marker can be uniformly scaled by setting the private `wrench_marker_scale` parameter.

To run it with a different input topic and/or scaling factor run.

```bash
rosrun wrench_marker wrench_marker_publisher wrench:=INPUT_TOPIC _wrench_marker_scale:=SCALING_FACTOR
```

Fine grained scaling control is applied (after initializing default values based on the general `wrench_marker_scale`) by reading from the following private parameters:

- `arrow_shaft_diameter`
- `arrow_head_diameter`
- `force_arrow_scale`
- `torque_arrow_scale`
- `torque_arrow_separation`


## `wrench_marker` library basic usage

Sample code for how to use the `wrench_marker` library follows.

```c++
#include <wrench_marker/wrench_marker.h>

int main( int argc, char** argv )
{
  wrench_marker::WrenchMarker my_wrench_marker;

  geometry_msgs::WrenchStamped wrench_msg;
  // Fill wrench data

  const visualization_msgs::MarkerArray& marker_array = my_wrench_marker.getMarker( wrench_msg );
  // Publish marker_array

  return 0;
}
```

A more ellaborate use-case can be found in [`wrench_marker_publisher.cpp`](src/wrench_marker_publisher.cpp)

### Reference

The full signature of the constructor is:

```c++
WrenchMarker::WrenchMarker(
  const WrenchMarkerScaleOptions scale_options = WrenchMarkerScaleOptions();
  const std::string& ns = "wrench",
  visualization_msgs::MarkerArray* msg = 0 );
```

- `scale_options` is an structure which can be used to control the scaling/positioning of the different arrows in the generated marker
- `ns` is the namespace for the markers
- `msg` is a pre-allocated `MarkerArray` message for internal use<sup>1</sup>

There are two overloads for `getMarker`.

The first one is:

```c++
const visualization_msgs::MarkerArray& WrenchMarker::getMarker(
  const geometry_msgs::WrenchStamped& wrench_stamped,
  const ros::Time& time = ros::Time::now() );
```

This will return a marker which will be displayed on top of the frame specified in `wrench_stamped.header.frame_id`.

The more complete one is:

```c++
const visualization_msgs::MarkerArray& WrenchMarker::getMarker(
  const geometry_msgs::Wrench& wrench,
  const std::string& frame,
  const geometry_msgs::Point& pos = geometry_msgs::Point(),
  const ros::Time& time = ros::Time::now() );
```

Here, the non stamped wrench is considered to be expressed in reference frame specified in `frame`, but referring to (and displayed at) the position given in `pos`.
Note that `pos` is considered to be relative to `frame`.

---

<sup>1</sup> Providing an allocated message is useful if used in conjunction with [`realtime_tools::RealtimePublisher`](https://github.com/ros-controls/realtime_tools).
