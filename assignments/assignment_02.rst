Assignment 2
############

.. .. attention::

..    You will need to update our repositories: :ref:`updating`.

Download the assignment PDF: :download:`Assignment 2 </res/pdfs/ws2425_assignment2.pdf>`

Install Dependencies
====================

The template provided for this assignment has additional dependencies.
We install them with

.. code-block:: console

   $ sudo apt install ros-jazzy-tf-transformations python3-transforms3d


New Simulation Launch Files
===========================

There are two more launch files in the ``fav`` package that we haven't used so far:

simulation_with_tags.launch.py
   Analogue to ``simulation.launch.py``, but additionally spawns landmark tags used for the distance-based localization.

simulation_with_tags_and_keyboard_control.launch.py
   Extends the above mentioned launch file by the keyboard control node.
   This is useful for testing your localization algorithms befor combining it with control nodes.

We can start the simulation with keyboard control with

.. code-block:: console

   $ ros2 launch fav simulation_with_tags_and_keyboard_control.launch.py vehicle_name:=bluerov00

Note the four AprilTags on the opposite side of the vehicle.

The measured distances to these tags are published under the ``ranges`` topic inside the vehicles namespace.
Details about the message definitions can be queried with

.. code-block:: console

   $ ros2 interface show hippo_msgs/msg/RangeMeasurementArray 
   std_msgs/Header header
      builtin_interfaces/Time stamp
         int32 sec
         uint32 nanosec
      string frame_id
   hippo_msgs/RangeMeasurement[] measurements
      std_msgs/Header header
         builtin_interfaces/Time stamp
            int32 sec
            uint32 nanosec
         string frame_id
      int32 id
      float64 range

Most relevant is the ``measurements`` array.
Each element of this array consists of a ``id`` field and a ``range`` field.
The ``id`` identifies the anchor/landmark/AprilTag to which the ``range`` (i.e. distance) was measured.
These IDs will be in the range [0;3] but the array of measurements is not guaranteed to be ordered.

If a landmark/AprilTag was not detected, it will not be included in the ``measurements`` array.
Thus, the ``measurements`` array is of variable size.

In Python we can iterate over the measurements with the following loop:

.. code-block:: python
   :linenos:
   
   for i, measurement in enumerate(msg.measurements):
      tag_id = measurement.id
      measured_distance = measurement.range
      self.get_logger().info(
          f'The {i}. element contains the measurement of tag {tag_id} with the '
          f'distance of {measured_distance}m')

.. seealso::

   A similar snippet can be found in the template code we provide for this assignment.

Template Package
================

Get the Template
****************

.. code-block:: console

   $ cd ~/fav/ros2/src && \
   git clone https://github.com/FormulasAndVehicles/position_control_template.git position_control

Launch Files
************

The template containts two launch files.

control.launch.py
   This starts the provided yaw controller.
   You can add your additional control nodes you implement for this assignment there.

   .. code-block:: console

      $ ros2 launch position_control control.launch.py vehicle_name:=bluerov00

   .. note:: 

      We recommend to start with the localization before taking care of control.
      Hence, you can leave this launch file alone for now and start it as soon as your localization procudes sufficient results.

localization.launch.py
   This node starts the Kalman Filter.

   .. code-block:: console

      $ ros2 launch position_control localization.launch.py vehicle_name:=bluerov00

Yaw Controller
**************

The template contains a very basic implementation of a P-controller for controlling the yaw angle.
You can find it in ``position_control/nodes/yaw_controller.py``.
The main purpose of the code is to provide an example on how to extract the vehicle's yaw angle from the ``vision_pose_cov`` topic.
Most likely you have already implemented a more advanced controller for the previous assignment.
Feel free to extend this controller or write a new one based on this base implemention as you see fit.

Kalman Filter
*************

This template contains the Kalman Filter that will compute a position estimate using the distance measurements to the tags/anchors.
You will need to implement the :code:`measurement_update` and the :code:`prediction` functions, as well as some data processing (in the callback function :code:`on_ranges`) beforehand.

The initial covariance matrices are all assumed to be diagonal. The measurement noise covariance matrix R and the prediction/process noise covariance matrix Q are used to tune your filter.
The diagonal entries are the squared standard deviations, i.e. for the process noise this means the first entry corresponds to how much the uncertainty in the x-position in each prediction step increases.
For your convenience, we have implemented the standard deviation as ROS parameters already.
To get more intuition about this, also have a look at the depth Kalman Filter example.


Ranges Debugger
***************

This is simply a convenience node. It republishes the range measurements in an ordered fashion under the topic :code:`/bluerov00/range_debugger/debug`. This allows you to plot the measurements from each tag individually.


The Distance Sensor
===================

The distance sensor is located at the front camera's position of the BlueROV, as depicted in :ref:`camera-sensors`.
Hence, the measurements are relative to this position.
Usually we consider the center of the vehicle as the robot's position.
It is fine to do the localization for the camera and apply the transformation to the robot's center in a post-processing step.

In the simulation the position of the distance sensor is exactly known and has an offset of ``[0.2, 0.0, 0.1]`` relative to the vehicle's origin.

Hints on the Controllers
========================

Feel free to reuse the depth controller from the previous assignment.
It can also be used as a base PID controller implementation for the additional controller(s).

It is up to you to decide wether you want to implement x-, y-, z-, and yaw-control in separate nodes each.
You might also find it more compelling to implement x- and y-control in the same node.

The yaw-controller is recommended, since the ``range_sensor`` can only detect the anchors/landmarks/AprilTags within a certain field of view.
Making the robot "looking" at the tags will make sure, they get detected more reliably.

Also keep in mind that there are almost no disturbances changing the BlueROV's heading in the simulation.
Most likely this will be different for the lab experiments, making the yaw-controller essential.

Additional Notes
================

Quaternions
***********

In this assignment we might get in touch of the rotation representation via quaternions.
The ROS convention is to write them in the order ``[x, y, z, w]``, while there is also the popular notation of writing them in alphabetical order ``[w, x, y, z]``.
We mention this here to avoid annoying mistakes caused by mixing these different notations.
Note, that in this assignment it will not be necessary to directly work with quaternions.
Since we are only interested in the yaw component, we will simply convert the orientation expressed in quaternions to an euler angle representation.

There are many euler-angle representations.
The one usually used for mobile robots is the intrinsic ``z-y'-x''``, which is equivalent to the extrinsic ``x-y-z``.
