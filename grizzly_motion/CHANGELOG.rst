^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grizzly_motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2014-12-18)
------------------
* Replace robot_pose_ekf with robot_localization.
* add front axle publisher.
* added publisher for wheel joints.
* Fix dependency on roboteq_msgs.
* Clarify timeout statemachine logic.
* Use the FindEigen from cmake_modules.
* Contributors: Mike Purvis, Shokoofeh Pourmehr

0.2.0 (2014-02-28)
------------------
* Change max acceleration from 0.5 to 1.5.
* Contributors: Mike Purvis

0.1.4 (2014-02-27)
------------------
* Disable encoder check pending a better implementation.
* Contributors: Mike Purvis

0.1.3 (2014-02-25)
------------------
* Check absolute value of measured speed; fixes bug with spurious encoder faults when driving backwards.
* Contributors: Mike Purvis

0.1.2 (2013-12-04)
------------------
* Install missing launch folder.

0.1.1 (2013-11-30)
------------------
* Add roboteq_msgs, nav_msgs dependency to grizzly_motion, remote unused eigen_conversions.

0.1.0 (2013-11-28)
------------------
* Initial release of motion package, corresponding to source release
  shipped with Nov 2013 deliveries. 
