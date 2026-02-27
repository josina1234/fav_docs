ROS Nodes: Subscribing and Checking Connections
###############################################

In this section you will

* access data from topics using a subscriber
* learn more about handling messages
* use the tool :code:`rqt_graph` for introspection and debugging

In :ref:`tutorials/ros_package:Write a Node`, you learned how to write a simple node.
While we learned how to publish data here, we still need to learn how to get access to data that is published within the ROS network.
This is called *subscribing* to a topic.

Get Data from Topics
====================

Robots typically have a lot of sensors that collect information about their environment.
This data can then be published within ROS.

Our BlueROV has a pressure sensor.
In the following, we will explore how to access the pressure data and how to process it in order to get the BlueROV's depth.
The output of the pressure sensor is published under the :file:`pressure` topic inside the vehicle's namespace.
So by default, the topic name will be :file:`/bluerov00/pressure`.

Theoretically, we could use the :file:`setpoint_publisher.py` and modify its code to subscribe to the :file:`pressure` topic.
But to keep things modular and separated, we add a new node to the :file:`awesome_package`.
Let's name it :file:`depth_calculator.py`.
You could argue that having a complete program to only calculate the depth coordinate of the vehicle from pressure data might seem like a bit overkill.
However, it is a good idea to solve separate problems in separate nodes.

.. note:: Keep in mind, you have to make every node executable! See :ref:`tutorials/ros_package:Write A Node`.

The source code of our node :file:`depth_calculator.py` might look like this:

.. code-block:: python
   :linenos:
   :caption: depth_calculator.py

   #!/usr/bin/env python3
   import rclpy

   from hippo_msgs.msg import DepthStamped
   from sensor_msgs.msg import FluidPressure
   from rclpy.node import Node


   class MySecondNode(Node):
       def __init__(self):
           super().__init__(node_name='my_second_node')
  
           # Create publisher for the calculated depth.
           self.depth_pub = self.create_publisher(DepthStamped, 'depth', 1)
  
           # Create subscriber. The third argument is the function that processes
           # the data. Every time fresh data is received, this function is called.
           # Subscribers should always come last within the __init__ function!
           self.pressure_sub = self.create_subscription(FluidPressure, 'pressure',
                                                        self.on_pressure, 1)

       def on_pressure(self, pressure_msg):
           pascal_per_meter = 1.0e4
           # Careful! You will have to do some additional thinking.
           # What kind of pressure data do we get? Relative/absolute?
           # What about the atmospheric pressure?
  
           depth = -pressure_msg.fluid_pressure / pascal_per_meter
  
           depth_msg = DepthStamped()
           depth_msg.depth = depth
           # let's add a timestamp:
           now = self.get_clock().now()
           depth_msg.header.stamp = now.to_msg()
  
           self.depth_pub.publish(depth_msg)
 
 
   def main():
       rclpy.init()
       node = MySecondNode()
       rclpy.spin(node)
 
 
   if __name__ == '__main__':
       main()


.. hint::
   Confused on how you should know what the structure of a :file:`FluidPressure` message looks like and how to access its data? 
   
   Simply google "ros fluidpressure" and you will find the `message definition <http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/FluidPressure.html>`_.
   Message fields are accessed by a dot operator.
   Message definitions can be *nested*, too.
   The `Header <http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html>`_ message within the FluidPressure message above is one example.
   
   All :file:`hippo_msgs` message definitions, such as the above-used :file:`DepthStamped` message, are our own definitions.
   You won't find these online.
   Instead, you can find the definition here: :file:`opt/ros/jazzy/share/hippo_msgs/msg/DepthStamped.msg`.

   More conveniently, you can always look up message definitions using the command line tool :code:`ros2 interface show`.
   
   For example
   
   .. code-block:: console

      $ ros2 interface show hippo_msgs/msg/DepthStamped

   will output:

   .. code-block:: console

      std_msgs/Header header
               builtin_interfaces/Time stamp
                  int32 sec
                  uint32 nanosec
               string frame_id

      float32 depth
      float32 z_vel
      float32 depth_covariance
      float32 vel_covariance


We can add this node to our launchfile as follows:

.. code-block:: python
   :linenos:
   :caption: ~/fav/ros2/src/awesome_package/launch/setpoint.launch.py
   :emphasize-lines: 22-23, 27


   from ament_index_python.packages import get_package_share_path
   from launch_ros.actions import Node, PushRosNamespace

   from launch import LaunchDescription
   from launch.actions import (
      DeclareLaunchArgument,
      GroupAction,
      IncludeLaunchDescription,
   )
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration


   def generate_launch_description() -> LaunchDescription:
       launch_description = LaunchDescription()
 
       arg = DeclareLaunchArgument('vehicle_name')
       launch_description.add_action(arg)
 
       setpoint_node = Node(executable='setpoint_publisher.py',
                            package='awesome_package')
       depth_node = Node(executable='depth_calculator.py',
                         package='awesome_package')
       group = GroupAction([
          PushRosNamespace(LaunchConfiguration('vehicle_name')),
          setpoint_node,
          depth_node,
       ])
       launch_description.add_action(group)
 
       package_path = get_package_share_path('fav')
       launch_path = str(package_path / 'launch/simulation.launch.py')
       source = PythonLaunchDescriptionSource(launch_path)
       launch_args = {'vehicle_name': LaunchConfiguration('vehicle_name')}
       action = IncludeLaunchDescription(source,
                                        launch_arguments=launch_args.items())
       launch_description.add_action(action)
 
       return launch_description

And launch the setup:

.. code-block:: console

   $ ros2 launch awesome_package setpoint.launch.py vehicle_name:=my_name



Inspecting the Setup
====================

So, our nodes are up and running. 
It will happen that things are not exactly working as they should, though. 

In the following, we will check and inspect our setup.
Are all nodes connected and interacting the way they should?

A very handy tool to inspect your setup is :file:`rqt_graph`.
Open another terminal to run

.. code-block:: console

   $ rqt_graph

Make sure to uncheck **Dead sinks** and **Leaf Topics**.
Also make sure **Nodes/Topics (all)** is selected in the upper left corner and refresh the view.
This should yield a graph like

.. image:: /res/images/tutorial_control_node_graph.png

You can see the different nodes :file:`/bluerov00/my_first_node` and :file:`/bluerov00/my_second_node`. 
We are not interested in a lot of the other nodes and will simply ignore them. 
Nodes are displayed as ellipses and topics as rectangles.
Since all these nodes live inside the :file:`/bluerov00` namespace and use relative topic names, everything has the :file:`/bluerov00` prefix.

The :file:`bluerov00/bridge` node is the interface between the (simulated in Gazebo) vehicle and our ROS domain.
On one hand, it receives the control commands that are then send to the thrusters.
On the other hand, it publishes sensor data, like the pressure sensor readings.

Regarding Control
*****************

We can send the following control setpoints to the :file:`bluerov00/actuator_mixer_node` node:

* Thrust setpoints: In the topic :file:`bluerov00/thrust_setpoint` using the message type :file:`hippo_msgs/ActuatorSetpoint`, we can send our **desired thrusts in x,y,z-direction** of the BlueROV.
* Torque setpoints: Similarly, in the topic :file:`bluerov00/torque_setpoint` using the message type :file:`hippo_msgs/ActuatorSetpoint`, we send the **desired torques around the x,y,z-axis** of the BlueROV.

That should be familiar to all of us from the previous tutorial and our dummy example with the :code:`setpoint_publisher.py`.
The message definition can be looked up in :file:`/opt/ros/jazzy/share/hippo_control_msgs/msg/ActuatorSetpoint.msg`  and is:

.. code-block::

   std_msgs/Header header

   bool ignore_x
   bool ignore_y
   bool ignore_z

   float64 x # [N] or [rad/s] or dimensionless effort in range [-1;1]
   float64 y # [N] or [rad/s] or dimensionless effort in range [-1;1]
   float64 z # [N] or [rad/s] or dimensionless effort in range [-1;1]

We can ignore the :code:`bool ignore_x` lines.
As an example, for full thrust in the x-direction of the BlueROV, we would publish a value of 1 for :code:`actuator_msg.x` and 0 for y and z.

Regarding Sensor Data
*********************

We can see that the (simulated) pressure sensor's data is also published by the :file:`bluerov00/bridge` node.
Our depth calculator, here still called :file:`my_second_node`, is connected to the sensor data!
Hooray!

However, we cannot say whether our calculations and their results actually look right...
Next, we will learn how to plot data.
