Assignment 1
############

Download the assignment PDF: :download:`Assignment 1 </res/pdfs/ws2425_assignment1.pdf>`

In this assignment, you will do depth control for our underwater robot.

In the hope of getting you started as smoothly as possible, we provide you with a template package for this assignment.
This way we skip the tasks of 

* creating a new package
* creating the required nodes
* setting up a basic subscription/publisher configuration

Additionally, you can start with the assignment with a working example (from a ROS point of view).
However, the code that actually solves the engineering/control tasks is not implemented in this template.
You will have to do it yourself.
The corresponding lines are marked with ``TODO`` comments.
But please feel free to tinker around with the rest of the code as well!

.. note::

   We cannot and also do not want to avoid learning how to create new packages/nodes during the course of this lecture.
   It it beneficial to start with it as soon as possible.
   Hence, we encourage you to read our guide on this topic and learn what it takes to write what the template already provides you with from scratch on your own.

Get the Template
================

We clone the repository and store it at ~/fav/ros2/src/depth_control

.. code-block:: console

   $ git clone https://github.com/FormulasAndVehicles/depth_control_template.git ~/fav/ros2/src/depth_control

As a general rule, if we have a new package we have to rebuild our workspace

.. code-block:: console

   $ build_ros

and source the workspace in all open terminal windows

.. code-block:: console

   $ source ~/.zshrc

The structure of the template looks similar to

.. code-block:: console

   ~/fav/ros2/src/depth_control
   ├── CMakeLists.txt
   ├── launch
   │   ├── depth_calculator.launch.py
   │   └── depth_control_full.launch.py
   ├── nodes
   │   ├── depth_calculator.py
   │   ├── depth_controller.py
   │   └── depth_setpoint.py
   └── package.xml

The Nodes Explained
===================

depth_calculator.py

   This node is equivalent to the node presented during the lecture.
   It subscribes the ``pressure`` topic of type ``FluidPressure`` and is responsible to calculate the depth of the vehicle based on this sensor input.
   In general, we are not restricted to a specific reference frame for expressing the depth coordinate.
   But in general, we recommend to define zero depth at water surface level and assume the positive direction to go upward.
   One could argue that we actually compute the height instead of the depth.
   But in the end, what is the difference between depth and height if not just a sign?

   ROS specifies a `convention <https://www.ros.org/reps/rep-0105.html>`__ for world-fixed coordinate frames. The z-coordinate (our depth) points upwards.

   .. note::

      Confused by the *quality of service* settings for the subscriber?
      For this specific topic a non-default quality of service profile is required.
      Otherwise the subscriber will not receive the messages.
      The reason behind this is maybe a bit out of scope of this assignment and it is perfectly fine if you ignore it.
      If you are still interested, we refer you to the `QoS concept page <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html>`__ of the ROS documentation.

depth_controller.py
   
   This node represents the controller that should control the vehicle's depth.
   It subscribes the ``depth`` topic of type ``DepthStamped`` and publishes a ``thrust_setpoint`` of type ``ActuatorSetpoint``.

   Maybe not as a first step, but the ability of dynamically tuning paramters could be quite useful at some point.
   You can look for available resources regarding ROS2 and reconfigurable parameters in the internet or you read the tutorial regarding this topic we are going to publish in the tutorials section of this website.

depth_setpoint.py

   This is just an example implementation of a node publishing setpoints for the depth calculator.
   In its current state, it publishes a square wave, i.e. the setpoints alternate between two distinct values with a fixed frequency.
   You can and should adapt this node to your needs. 
   Maybe you want to change the values or use another kind of setpoint function (constant, sine, or something completely different)?
   A controller that copes very well with constant setpoints might perform really badly for dynamic septoints.
   Moreover, there are clearly limits to the vehicle's dynamics.
   It cannot change its velocity arbitrarily fast, as we might remember from the mechanics lectures.

The Launch Files
================

depth_calculator.launch.py

   Runs the ``depth_calculator`` node that is to be implemented in assignment 1a.

depth_control_full.launch.py

   This file enables us to start all the nodes at the same time.
   This becomes mainly useful for assignment 1b and following.
   We at least encountered three launch files during assignment 0, even though we do not know mouch about them yet.
   It is perfectly fine if it stays that way for now.
   But at some time in the future you will probably like to read something about them.
   You can do so in our tutorials section.
   Besides that, there is a plethora of great tutorials regarding launch files (and actually almost everything else regarding ROS) just one google search away.

Get Going
=========

Assignment 1a
-------------

We start the simulation with the following command

.. code-block:: console

   $ ros2 launch fav simulation_with_keyboard_control.launch.py vehicle_name:=bluerov00 use_sim_time:=true

and start the depth calculator node from the template package with

.. code-block:: console

   $ ros2 launch depth_control depth_calculator.launch.py vehicle_name:=bluerov00 use_sim_time:=true

Assignment 1b
-------------

We start the simulation with the following command

.. code-block:: console

   $ ros2 launch fav simulation.launch.py vehicle_name:=bluerov00 use_sim_time:=true

and start our depth control setup in a second terminal with

.. code-block:: console

   $ ros2 launch depth_control depth_control_full.launch.py vehicle_name:=bluerov00 use_sim_time:=true

.. note::

   In contrast to task 1a, we do not want the simulation with the keyboard control.
   We will have our depth control node, that is responsible for publishing desired thrust commands.
   If we would start the keyboard control node at the same time, they would publish different values to the same topic.
   The result will be most likely chaotic.

.. hint::

   Have you installed ``terminator`` according to :ref:`installation/linux_terminal:Linux Terminal` ?
   In terminator, you can create subterminals by rightclicking inside the terminal and choose to split horizontally or vertically.
   This way you do not have to open two or more separate terminals.

   Wanna feel very hacky? 
   Then you will probably throw away your mouse in favour of your keyboard.
   The splits are created with :kbd:`Ctrl` + :kbd:`E` and :kbd:`Ctrl` + :kbd:`O`.
   You can navigate through the terminals with :kbd:`Alt` + :kbd:`ArrowKeys`.

You should receive many logs from the second launch setup regarding received messages.
If no regular log messages appear, this indicates that something is not working properly.
If you are not able to fix the problem yourself, you may want to ask your favourite research associate for help!

Additional Notes
================

Keyword Arguments
*****************

The concept of keyword arguments might be new for you.
Instead of passing arguments to a function in a specific order, we can use keywords to assign values to the parameters.
Often, this is optional.
But there are, as so often in life, some exceptions.
We used keyword arguments instead of positional arguments quite often in the template to make the code a bit more verbose.

For example, the following line

.. code-block:: python

   self.create_publisher(msg_type=FluidPressure, topic='pressure', qos_profile=1)

makes it quite easy to understand the meaning of the parameters we pass to the function.
The order does not matter in this case, since we e.g. explicitly assign ``'pressure'`` to the ``topic`` function parameter.

If we would pass the aruments as positional arguments, the line would look like

.. code-block:: python

   self.create_publisher(FluidPressure, 'pressure', 1)

This time the order of the arguments matters.
The advantage of this way to pass the arguments is that its much more compact.
But this comes with less verbosity.
Someone reading the code would have to lookup the parameters of ``create_publisher()`` before being able to understand their meaning.

You can stick with the variant you prefer.
You will probably see the latter variant more often, though.

.. note::

   There are also functions with positional arguments that you can optionally pass by their keywords, and arguments that you can **exclusively** pass by keywords.

Try-Except Around rclpy.spin
****************************

Are you wondering why we have these blocks

.. code-block:: python

   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass

around ``rclpy.spin(node)``?
Glad you asked!
When we hit :kbd:`Ctrl` + :kbd:`C` to stop the execution of our programs, a keyboard interrupt is triggered.
This tells our program to stop.
As a consequence, an exception is raised. 
If this exception was not handled, we would get a not very useful terminal output.
By catching the exception and doing nothing (indicated by ``pass``) we supress this output.

