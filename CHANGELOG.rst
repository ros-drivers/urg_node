^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urg_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.18 (2022-09-29)
-------------------
* Encode sent msg to bytes object when comparing with returned msg bytes object from socket; ensures compatability with Python 2 and Python 3; add comments.
* Install Python helper script using catkin_install_python() so the correct Python version is selected automatically; ensure socket sends message as byte object for Python3, while ensuring backwards compatability with Python2.
* Contributors: Joey Yang

0.1.17 (2022-03-03)
-------------------
* Update UST10 (`#96 <https://github.com/ros-drivers/urg_node/issues/96>`_)
  * Fix typo for package name
  * Replace UST10 URDF with one used in CPR robots ; remove lx suffix from UST10 files
  * Fix typo in package name
  * Add installation of urdf, meshes, and launch directories to CMakeLists.txt
* Add collision to URDF
* Add URDF and STL of Hokuyo UST-10LX
* Contributors: Joey Yang

0.1.16 (2022-01-27)
-------------------
* roslint fixes.
* Diagnostic Analyzers (`#93 <https://github.com/ros-drivers/urg_node/issues/93>`_)
  * Added diagnostic analyzers to organize robot_monitor
  * Update Change Log
  * Moved addDiagnostics call to the diagnostics thread
  * Changed parameter prefix from "/" to ""
  * Removed edits to the CHANGELOG
* Contributors: Tony Baltovski, luis-camero

0.1.15 (2020-10-27)
-------------------
* Function setSkip() set as void
  This function as no return type causing undefined behavior. This function
  has been declared as void.
* Contributors: bostoncleek

0.1.14 (2020-06-04)
-------------------
* Bumped CMake version.
* Removed trailing whitespace.
* Contributors: Tony Baltovski

0.1.13 (2020-03-27)
-------------------
* Revert "fix(updateStatus): Update status on diagnostics update"
* Contributors: Tony Baltovski

0.1.12 (2020-03-14)
-------------------
* Updated roslint to only check files in this repo.
* Updated TravisCI config.
* fix(updateStatus): Update status on diagnostics update
  Otherwise the diagnostics information does not really reflect the device
  status.
* synchronize_time: reset when clock is warped
  If either the hardware clock or system clock warp, reset the EMA to
  prevent incorrect clock values from being used. Detect the warp by
  putting a limit on the absolute error between the synchronized clock
  and the system clock. When a warp is detected, reset the EMA to force
  the clocks to resynchronize. Use the system clock until the EMA has
  stabalized again.
* synchronize system clock to hardware time
  Remove jitter from the system clock by synchronizing it to the change
  in hardware time stamps. This does not synchrnoize it in an absolute
  sense (i.e., doesn't remove system latench). However, coupled with
  calibrating system latency, this results in a stable, accurate clock.
* Add Travis config.
* Fixed linter errors.
* Contributors: C. Andy Martin, Rein Appeldoorn, Tony Baltovski

0.1.11 (2017-10-17)
-------------------
* Add support for URG-04LX in SCIP 1.1 mode
  The urg_node does not support SCIP 1.1. The Hokuyo URG-04LX supports both
  SCIP 1.1 and SCIP 2.0, but needs to be switched to SCIP 2.0 at every startup
  in its default configuration. For this purpose the function
  URGCWrapper::setToSCIP2() was added.
  A URG-04LX in SCIP 1.1 mode used to lead to an exception being thrown in
  URGCwrapper::initialize. Now, before throwing the exception an attempt to
  switch the sensor to SCIP 2.0 is made.
* Fixed comments in launch file and added roslaunch.
* Add flag to prevent updating of detailed status.
  If using a model that does not support AR00 command, hide it
  behind a rosparam.
* Add safety stop heading and distance values (`#28 <https://github.com/ros-drivers/urg_node/issues/28>`_)
  Added to the laser status field the last report of a safety
  stop of distance and angle reported. If this fails or is unavailable
  it will just report 0.
* Updating depend and roslint.
  Fixing some roslint error after moving a header name.
  Additionally fixing the gencfg to be on the lib and not the node.
* Adding missing std_srvs depend.
  Adding missing std_srvs depend to package.xml and CMakelists.txt
* Move urg_node to be a library.
  Moving urg_node to urg_node_driver as a library.
  This allows for other nodes to include this as an object instead
  of spawning another separate process.
* Add getAR00 status command.
  Added ability to pull the status of the lidar AR00 status command.
  This then publishes a latched topic with the current status of the
  lidar's error code and lockout status.
* Update urg_node to be a self contained class
  Updating urg node to be a self contained class. This allows
  for it to be imported in other nodes.
* Roslint
* Contributors: Benjamin Scholz, Mike O'Driscoll, Tony Baltovski

0.1.10 (2017-03-21)
-------------------
* Updated maintainer.
* Error handling for connection failures
* Created urg_lidar.launch
* Installed getID
* Contributors: Eric Tappan, Jeff Schmidt, Kei Okada, Tony Baltovski

0.1.9 (2014-08-13)
------------------
* Merge pull request `#7 <https://github.com/ros-drivers/urg_node/issues/7>`_ from mikeferguson/indigo-devel
  add a script to set the IP address of an URG laser
* Updated diagnostics to support configurable parameters.
* add a script to set the IP address of an URG laser
* Contributors: Chad Rockey, Michael Ferguson

0.1.8 (2014-06-16)
------------------
* Merge pull request `#6 <https://github.com/ros-drivers/urg_node/issues/6>`_ from mikeferguson/indigo-devel
  Add default device status on UST-20LX
* Add default device status on UST-20LX
* Contributors: Chad Rockey, Michael Ferguson

0.1.7 (2014-04-21)
------------------
* Added more robust plug/unplug reconnect behavior.
* Added more robustness and the ability to continually reloop and reconnect until node is shutdown.
* Fix initialization crash.
* Install fix for Android.
* Missed a willowgarage email.
* Contributors: Chad Rockey

0.1.6 (2013-10-24)
------------------
* Added getID executable for udev users.

0.1.5 (2013-08-22)
------------------
* Missing diagnostic_updater depend

0.1.4 (2013-08-22)
------------------
* Merge pull request `#2 <https://github.com/ros-drivers/urg_node/issues/2>`_ from mitll-ros-pkg/diagnostics
  Added diagnostics to the URG Node.
* Added diagnostics to the URG Node.

0.1.3 (2013-08-21)
------------------
* No more Willow Garage email.

0.1.2 (2013-03-14)
------------------
* Be more tolerant of connection dropouts and try to reconnect.
* Fixed poor initilization causing uncertain output.
* Updated consts

0.1.1 (2013-03-04)
------------------
* Only advertise for single or multiecho, not both.
* Generalized multi echo grab function
* Updated to use laser_proc to automatically publish compatibility messages.
* Optimize the fill multi echo laserscan message to use reserve instead of resize wherever possible.

0.1.0 (2013-03-03)
------------------
* Added install rules.
* Renamed package to urg_node.
* Updated to use better timestamping.
* Updated to use urg_c name for library.
* Added information functions for future diagnostics.
* Added experimental timestamp synchronization.  Fixed segfault for multiecho intensity.
* Fixed skip being cluster.  Added skip functionallity.
* Connected dynamic reconfigure, including angle limit requests.
* Added ability to publish both single and multi echo scans.
* Added dynamic reconfigure; can update reconfigure limits
* Initial commit.  Connecting to both ethernet and serial devices.
* Initial commit
