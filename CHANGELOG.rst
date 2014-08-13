^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urg_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
