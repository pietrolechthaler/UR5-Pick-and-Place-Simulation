^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2022-05-10)
------------------

3.0.2 (2022-04-05)
------------------
* Depend on orocos_kdl_vendor (`#191 <https://github.com/ros/robot_state_publisher/issues/191>`_)
* Contributors: Jacob Perron

3.0.1 (2022-03-28)
------------------
* export dependencies, to use robot_state_publisher as a component (`#193 <https://github.com/ros/robot_state_publisher/issues/193>`_)
* Contributors: Kenji Brameld

3.0.0 (2022-01-14)
------------------
* Fix include order for cpplint (`#186 <https://github.com/ros/robot_state_publisher/issues/186>`_)
* Change how parameter updates are handled (`#180 <https://github.com/ros/robot_state_publisher/issues/180>`_)
* Install includes to instal/${PROJECT_NAME} (`#184 <https://github.com/ros/robot_state_publisher/issues/184>`_)
* Make the change_fixed_joint test more robust (`#183 <https://github.com/ros/robot_state_publisher/issues/183>`_)
* Add in a test to make sure fixed transforms change on update
* Small C++ nice-isms in the tests
* Switch to using target_include_directories for tests
* Publish new fixed transforms when URDF is updated
* Make joint_states subscription QoS configurable; default to SensorDataQoS (`#179 <https://github.com/ros/robot_state_publisher/issues/179>`_)
* Remove dependency on urdfdom_headers (`#168 <https://github.com/ros/robot_state_publisher/issues/168>`_)
* Contributors: Anthony Deschamps, Chris Lalancette, Jacob Perron, Russell Joyce, Shane Loretz

2.7.0 (2021-10-18)
------------------
* Fix deprecated subscriber callbacks (`#173 <https://github.com/ros/robot_state_publisher/issues/173>`_)
* Contributors: Abrar Rahman Protyasha

2.6.0 (2021-08-02)
------------------
* Cleanup the documentation in the RobotStatePublisher class. (`#172 <https://github.com/ros/robot_state_publisher/issues/172>`_)
* Always publish fixed frames to /tf_static (`#158 <https://github.com/ros/robot_state_publisher/issues/158>`_)
* corrected publish_frequency default in README (`#166 <https://github.com/ros/robot_state_publisher/issues/166>`_)
* Contributors: Chris Lalancette, Jacob Perron, Nils Schulte

2.5.0 (2021-06-11)
------------------
* Add tf frame_prefix parameter (`#159 <https://github.com/ros/robot_state_publisher/issues/159>`_)
* Contributors: Steve Nogar, Chris Lalancette

2.4.3 (2021-04-19)
------------------
* Stop rejecting unknown parameters. (`#161 <https://github.com/ros/robot_state_publisher/issues/161>`_)
* Contributors: Chris Lalancette

2.4.2 (2021-01-25)
------------------
* clean up license to be standard bsd 3 clause (`#130 <https://github.com/ros/robot_state_publisher/issues/130>`_)
* Update the maintainers. (`#151 <https://github.com/ros/robot_state_publisher/issues/151>`_)
* Contributors: Chris Lalancette, Tully Foote

2.4.1 (2020-09-28)
------------------
* fix types in range loops to avoid copy due to different type (`#143 <https://github.com/ros/robot_state_publisher/issues/143>`_)
* Make sure not to crash on an invalid URDF. (`#141 <https://github.com/ros/robot_state_publisher/issues/141>`_)
* Contributors: Chris Lalancette, Dirk Thomas

2.4.0 (2020-04-30)
------------------
* Replace deprecated launch_ros usage (`#137 <https://github.com/ros/robot_state_publisher/issues/137>`_)
* code style only: wrap after open parenthesis if not in one line (`#129 <https://github.com/ros/robot_state_publisher/issues/129>`_)
* Refactor the ROS 2 code to be more modern (`#126 <https://github.com/ros/robot_state_publisher/issues/126>`_)
* Switch to using TARGET_FILE to find the binary targets on Windows.
* Fix tests on Windows.
* Make the error message much nicer on Windows.
* robot_state_publisher_node -> robot_state_publisher
* Fix test build on Windows.
* Get rid of redundant declaration.
* Guard against negative publish_frequencies.
* Switch to modern launch_testing ReadyToTest.
* Add testing to robot_state_publisher.
* Update some example launch files.
* Implement the ability to change the parameters on the fly.
* Fix silly bug while computing the publish interval.
* Refactor to have a "setup" function during the constructor and later on during the parameter setup.
* Mark things as explicit and final.
* Update the documentation.
* Make robot_state_publisher composable.
* Contributors: Chris Lalancette, Dirk Thomas, Jacob Perron

2.3.1 (2019-10-23)
------------------
* Switch the license back to BSD. (`#121 <https://github.com/ros/robot_state_publisher/issues/121>`_)
* Contributors: Chris Lalancette

2.3.0 (2019-09-26)
------------------
* Install include directories (`#31 <https://github.com/ros2/robot_state_publisher/issues/31>`_)
* Publish URDF string on startup (`#24 <https://github.com/ros2/robot_state_publisher/issues/24>`_)
* Contributors: Patrick Beeson, Poh Zhi-Ee, Shane Loretz

2.2.4 (2019-09-06)
------------------
* Remove unused Eigen3 dependency (`#27 <https://github.com/ros2/robot_state_publisher/issues/27>`_) (`#29 <https://github.com/ros2/robot_state_publisher/issues/29>`_)
* Don't export exe as library (`#25 <https://github.com/ros2/robot_state_publisher/issues/25>`_) (`ros2 #28 <https://github.com/ros2/robot_state_publisher/issues/28>`_)
* Contributors: Shane Loretz

2.2.3 (2019-06-12)
------------------
* Use rclcpp::Time for stamping transforms (`#20 <https://github.com/ros2/robot_state_publisher/issues/20>`_)
* Contributors: Scott K Logan

2.2.2 (2019-05-08)
------------------
* changes to avoid deprecated API's (`#19 <https://github.com/ros2/robot_state_publisher/issues/19>`_)
* Contributors: William Woodall

2.2.1 (2019-04-26)
------------------
* Updated to avoid deprecated API. (`#18 <https://github.com/ros2/robot_state_publisher/issues/18>`_)
* Contributors: William Woodall

2.2.0 (2019-04-14)
------------------
* Set urdf content as parameter. (`#15 <https://github.com/ros2/robot_state_publisher/issues/15>`_)
* Contributors: Karsten Knese

2.1.0 (2018-06-27)
------------------
* The robot model is now published on the ``robot_description`` topic as a ``std_msgs/String.msg``. (`#9 <https://github.com/ros2/robot_state_publisher/issues/9>`_)
* Contributors: Brett, Mikael Arguedas

1.13.4 (2017-01-05)
-------------------
* Use ``urdf::*ShredPtr`` instead of ``boost::shared_ptr`` (`#60 <https://github.com/ros/robot_state_publisher/issues/60>`_)
* Error log for empty JointState.position was downgraded to a throttled warning (`#64 <https://github.com/ros/robot_state_publisher/issues/64>`_)
* Contributors: Jochen Sprickerhof, Sébastien BARTHÉLÉMY

1.13.3 (2016-10-20)
-------------------
* Added a new parameter "ignore_timestamp" (`#65 <https://github.com/ros/robot_state_publisher/issues/65>`_)
* Fixed joints are not published over tf_static by default (`#56 <https://github.com/ros/robot_state_publisher/issues/56>`_)
* Fixed segfault on undefined robot_description (`#61 <https://github.com/ros/robot_state_publisher/issues/61>`_)
* Fixed cmake eigen3 warning (`#62 <https://github.com/ros/robot_state_publisher/issues/62>`_)
* Contributors: Davide Faconti, Ioan A Sucan, Johannes Meyer, Robert Haschke

1.13.2 (2016-06-10)
-------------------
* Add target_link_libraries for joint_state_listener library + install it (`#54 <https://github.com/ros/robot_state_publisher//issues/54>`_)
* Contributors: Kartik Mohta

1.13.1 (2016-05-20)
-------------------
* Add back future dating for robot_state_publisher (`#49 <https://github.com/ros/robot_state_publisher/issues/49>`_) (`#51 <https://github.com/ros/robot_state_publisher/issues/51>`_)
* Fix subclassing test (`#48 <https://github.com/ros/robot_state_publisher/issues/48>`_)
* Support for subclassing (`#45 <https://github.com/ros/robot_state_publisher/issues/45>`_)
  * Add joint_state_listener as a library
* Contributors: Jackie Kay

1.13.0 (2016-04-12)
-------------------
* fix bad rebase
* Contributors: Jackie Kay, Paul Bovbel

1.12.1 (2016-02-22)
-------------------
* Merge pull request `#42 <https://github.com/ros/robot_state_publisher/issues/42>`_ from ros/fix_tests_jade
  Fix tests for Jade
* Correct failing tests
* Re-enabling rostests
* Merge pull request `#39 <https://github.com/ros/robot_state_publisher/issues/39>`_ from scpeters/issue_38
* Fix API break in publishFixedTransforms
  A bool argument was added to
  RobotStatePublisher::publishFixedTransforms
  which broke API.
  I've added a default value of false, to match
  the default specified in the JointStateListener
  constructor.
* Contributors: Jackie Kay, Jonathan Bohren, Steven Peters

1.12.0 (2015-10-21)
-------------------
* Merge pull request `#37 <https://github.com/ros/robot_state_publisher/issues/37>`_ from clearpathrobotics/static-default
  Publish fixed joints over tf_static by default
* Merge pull request `#34 <https://github.com/ros/robot_state_publisher/issues/34>`_ from ros/tf2-static-jade
  Port to tf2 and enable using static broadcaster
* Merge pull request `#32 <https://github.com/ros/robot_state_publisher/issues/32>`_ from `shadow-robot/fix_issue#19 <https://github.com/shadow-robot/fix_issue/issues/19>`_
  Check URDF to distinguish fixed joints from floating joints. Floating joint are ignored by the publisher.
* Merge pull request `#26 <https://github.com/ros/robot_state_publisher/issues/26>`_ from xqms/remove-debug
  get rid of argv[0] debug output on startup
* Contributors: David Lu!!, Ioan A Sucan, Jackie Kay, Max Schwarz, Paul Bovbel, Toni Oliver

1.11.1 (2016-02-22)
-------------------
* Merge pull request `#41 <https://github.com/ros/robot_state_publisher/issues/41>`_ from ros/fix_tests_indigo
  Re-enable and clean up rostests
* Correct failing tests
* Re-enabling rostests
* Fix API break in publishFixedTransforms
  A bool argument was added to
  RobotStatePublisher::publishFixedTransforms
  which broke API.
  I've added a default value of false, to match
  the default specified in the JointStateListener
  constructor.
* Contributors: Jackie Kay, Jonathan Bohren, Steven Peters

1.11.0 (2015-10-21)
-------------------
* Merge pull request `#28 <https://github.com/ros/robot_state_publisher/issues/28>`_ from clearpathrobotics/tf2-static

1.10.4 (2014-11-30)
-------------------
* Merge pull request `#21 <https://github.com/ros/robot_state_publisher/issues/21>`_ from rcodddow/patch-1
* Fix for joint transforms not being published anymore after a clock reset (e.g. when playing a bagfile and looping)
* Contributors: Ioan A Sucan, Robert Codd-Downey, Timm Linder

1.10.3 (2014-07-24)
-------------------
* add version depend on orocos_kdl >= 1.3.0
  Conflicts:
  package.xml
* Update KDL SegmentMap interface to optionally use shared pointers
  The KDL Tree API optionally uses shared pointers on platforms where
  the STL containers don't support incomplete types.
* Contributors: Brian Jensen, William Woodall

1.10.0 (2014-03-03)
-------------------
* minor style fixes
* Add support for mimic tag.
* Contributors: Ioan Sucan, Konrad Banachowicz
