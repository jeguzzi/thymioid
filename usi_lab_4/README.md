Assignment 4: ROS
===================

Installation
------------

1. install the ROS depencies

  ```bash
  sudo apt-get install ros-indigo-ar-track-alvar
  sudo apt-get install ros-indigo-keyboard
  ```

2. install the Thymio ROS packages

  ```bash
  cd <your_catkin_ws>/src
  git clone -b client https://github.com/jeguzzi/ros-aseba.git
  git clone -b client https://github.com/jeguzzi/thymioid.git
  ```

3. compile

  ```bash
  cd <your_catkin_ws>
  catkin_make
  ```


Running the simulation
---------------------

1. setup the environment (for every shell). You may want to add the line below to your .bashrc file.

  ```bash
  source <your_catkin_ws>/devel/setup.bash
  ```

2. launch roscore

  ```bash
  roscore
  ```

3. launch vrep

  ```bash
  sh <your-vrep-folder>/vrep.sh
  ```

4. open the scene located in `<your_catkin_ws>/src/thymioid/usi_lab_4/scene/lab4.skeleton`

5. start the simulation

6. launch the Thymio controller

  ```bash
  roslaunch usi_lab_4 skelethon.launch
  ```

7. (optional) launch rviz to visualize the information the robot is collecting

  ```bash
  roslaunch thymoid rviz.launch
  ```




Notes
----

You may need to kill and restart
`roslaunch usi_lab_4 skeleton.launch`
every time you start a new simulation.


`skelethon.launch` launches the basic nodes to


* control the thymio,
* perform the tracking of the markers,

together with a controller to steer the thymio using the keyboard keys (press space to stop). To control the thymio with the keyboard, the small keyboard window should be in focus.


Sensing
-------

The most important topics that carry sensing informations are

* `/odom` from which you can read the current linear and angular speeds, see the [message format](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* `/proximity/<name>` from which you can read the range of the nearest obstacles sensed by the proximity sensor <name>, see the [message format](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)
* `/ar_pose_marker`, from which you can read the pose of the main face of the visible cubes in the camera frame, see the [message format](http://docs.ros.org/api/ar_track_alvar/html/msg/AlvarMarkers.html)

### Markers

Each cube is covered by unique visual markers that are detectable by the robot using its on-board
camera and the ROS package [`ar_track_alvar`](http://wiki.ros.org/ar_track_alvar), which publishes the id and pose of one of the faces of the cube based on the xml descriptions contained in /bundles. Each pose is given with respect to the camera frame denoted as `camera_link`. Moreover, for every detected cube, the node publishes a [tf](http://wiki.ros.org/tf) coordinates frame. tf is a ROS package that takes care of transforming between the various coordinate frame.

To get the range and bearing of the center of the cube with respect to the robot center, you should first use tf to get the pose in the robot frame `base_link`, as explained [here](http://wiki.ros.org/tf/Overview/Transformations) and [here](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF). For example, using C++, the following ROS node print the polar coordinates of recent markers:

```c++
/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#define CUBE_HALF_SIZE 0.025

class PoseNode
{
  ros::Subscriber pose_sub_;
  ros::NodeHandle nh_;
  tf::TransformListener listener;
public:

  PoseNode()
    :listener(ros::Duration(10))
  {
    pose_sub_ = nh_.subscribe("ar_pose_marker", 1, &PoseNode::updatePose, this);
}

  tf::Point positionInRobotFrame(geometry_msgs::PoseStamped face_pose_in_camera_frame)
    {
        geometry_msgs::PoseStamped cube_pose_in_robot_frame;
        geometry_msgs::PoseStamped cube_pose_in_camera_frame=face_pose_in_camera_frame;
        //move from the face center to the center of the cube.
        //The z-axis of the marker frame points outwards the face.
        cube_pose_in_camera_frame.pose.position.z-=CUBE_HALF_SIZE;
        try{
            listener.transformPose("base_link",cube_pose_in_camera_frame,cube_pose_in_robot_frame);
            return tf::Point(cube_pose_in_robot_frame.pose.position.x,
                             cube_pose_in_robot_frame.pose.position.y,
                             cube_pose_in_robot_frame.pose.position.z);
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"base_link\": %s",                face_pose_in_camera_frame.header.frame_id.c_str(),  ex.what());
            return tf::Point(0,0,0);
        }
    }


  void updatePose(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg)
  {
    std::vector<ar_track_alvar_msgs::AlvarMarker> markers=msg->markers;
    for(int i=0;i<markers.size();i++)
      {
	ar_track_alvar_msgs::AlvarMarker marker=markers[i];
	int id=marker.id;
	geometry_msgs::PoseStamped pose=marker.pose;
	pose.header.frame_id=marker.header.frame_id;
	tf::Point p=positionInRobotFrame(pose);
	printf("Cube %d is located at (%.2f %.2f) in the robot frame\n",id,p.x(),p.y());
	double beta=atan2(p.y(),p.x());
	double rho=sqrt(p.x()*p.x() +p.y()*p.y());
	printf("i.e. at rho %.2f and beta %.2f\n",rho,beta);
      }

}
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_node");
  PoseNode ic;
  ros::spin();
  return 0;
}
```


Finally, a simple transformation from Cartesian to polar coordinates gives you the range and bearing of the cube.


Robot Model
----------

To locate a marker in the 3D world, the robot needs to have a model of the marker and of itself.
The later is contained in the file thymioid/models/thymioid.urdf.xacro. If you change the pitch of the camera, you should also change the robot model and, in particular, update the orientation of the  `camera_body_support_joint`
```xml
<joint name="camera_body_support_joint" type="fixed">
    <origin xyz="0.015 0 0" rpy="0 0.2618 0"/>
    <parent link="camera_support_link"/>
    <child link="camera_body_link"/>
</joint>
```

which by default is tilted by 15 degrees (i.e. 0.2618 rad).
