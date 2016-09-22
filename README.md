# Wrench Marker

This package contains both a library and an executable node.

The `wrench_marker_publisher` node can be used to generate marker messages out of wrenches published to a ROS topic.

It internally uses the `wrench_marker` library.
This library exports the `wrench_marker/wrench_marker.h` header, in which a single class is declared.
It can be used to generate rviz markers (i.e. `visualization_msgs/MarkerArray` messages) from wrenches (i.e. `geometry_msgs/Wrench` messages) that use several arrow markers to display the wrench in rviz.
The responsibility of publishing the message lies in user code.

![Wrench marker on LWR](http://i.imgur.com/9a1wh65.png)

## `wrench_marker_publisher` node

The `wrench_marker_publisher` node subscribes to `geometry_msgs/Wrench` messages in the `wrench` topic, and publishes a marker array to visualize it in `ẁrench_marker` topic.

If `wrench_marker_frame` private parameter exists, it will use its value as the reference frame for the published markers.
Otherwise, it will default to using the `world` frame.

To run it with a different input topic and using a specific reference frame for the marker, run.

```bash
rosrun wrench_marker wrench_marker_publisher wrench:=INPUT_TOPIC _wrench_marker_frame:=MARKER_FRAME
```

## `wrench_marker` library basic usage

The only header for the library can be included (provided the proper dependencies are added to your `package.xml` and the include path is set in your `CMakeLists.txt`) with

```c++
#include <wrench_marker/wrench_marker.h>
```

Initialize the class with the name of the reference frame for displaying the wrench marker.

```c++
wrench_marker::WrenchMarker my_wrench_marker( "world" );
```

Then a marker array can be populated with the right markers with:

```c++
geometry_msgs::Wrench wrench_msg;
// add data to wrench_msg...

const visualization_msgs::MarkerArray& msg = my_wrench_marker.populate( wrench_msg );
```

Note: It's up to the library user to later publish this message in any desired way.

Building of your executable/library can then proceed as usual, linking against the exported catkin libraries.

### Reference

The full signature of the constructor is:

```c++
WrenchMarker( 
    const std::string& frame,
    const std::string& ns = "wrench",
    visualization_msgs::MarkerArray* msg = 0 );
```

- `ns` is the namespace for the individual markers in the array
- `msg` is an already allocated `MarkerArray` message for internal use<sup>1</sup>

The full signature for `populate` function is:

```c++
const visualization_msgs::MarkerArray& WrenchMarker::populate(
    const geometry_msgs::Wrench& wrench,
    const ros::Time& time = ros::Time::now(),
    const geometry_msgs::Point& pos = geometry_msgs::Point() );
```

- `time` takes a const reference to a `ros::Time`, which is used to timestamp the individual markers in the array
- `point` takes a const reference to a `geometry_msgs::Point`, which is the position where the markers will be drawn

---

<sup>1</sup> Providing an allocated message is useful if used in conjunction with [`realtime_tools::RealtimePublisher`](https://github.com/ros-controls/realtime_tools).