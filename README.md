# Wrench Marker

This package exports the `wrench_marker/WrenchMarker.h` header, which defines an utility class to generate rviz markers to display wrenches using several arrow markers inside a `visualization_msgs::MarkerArray` message.

![Wrench marker on LWR](http://i.imgur.com/9a1wh65.png)

## Basic usage

Initialize the class with the name of the reference frame for the wrench.

```c++
WrenchMarker wrench_marker( "world" );
```

Then a marker array can be populated with the right markers with:

```c++
geometry_msgs::Wrench wrench_msg;
// add data to wrench_msg...

const visualization_msgs::MarkerArray& msg = wrench_marker.populate( wrench_msg );
```

## Reference

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
const visualization_msgs::MarkerArray& populate(
    const geometry_msgs::Wrench& wrench,
    const ros::Time& time = ros::Time::now(),
    const geometry_msgs::Point& pos = geometry_msgs::Point() );
```

- `time` takes a const reference to a `ros::Time`, which is used to timestamp the individual markers in the array
- `point` takes a const reference to a `geometry_msgs::Point`, which is the position where the markers will be drawn

---

<sup>1</sup> Providing an allocated message is useful if used in conjunction with [`realtime_tools::RealtimePublisher`](https://github.com/ros-controls/realtime_tools).