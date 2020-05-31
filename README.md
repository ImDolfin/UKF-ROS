# see_ego_motion

This node implements basic ROS communication interfaces.

## Usage

After starting `roscore` in another terminal, execute the following commands.

  `cd rosdir`\
  `catkin_make`\
  `rosrun see_ego_motion see_ego_motion`

## Nodes

### see_ego_motion

Offers publication, subscription and service interfaces.

#### Subscribed Topics

* **`see_ego_motion_subscription`** [std_msgs/String]

  Receives a string and does nothing.

#### Published Topics

* **`see_ego_motion_publication`** [std_msgs/String]

  Publishes a string with a frequency of 1 Hz.

#### Services

* **`see_ego_motion_service`** [std_srvs/Trigger]

  Can be polled for string retrieval.
