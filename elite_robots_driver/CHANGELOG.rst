^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package elite_robots_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2026-08-31)
------------------
* Link SDK consumers against the exported ``elite-cs-series-sdk::shared`` CMake target.
* Propagate SDK usage requirements through exported driver components.
* Contributors: Yan

1.0.2 (2026-08-28)
------------------
* Use the SDK ROS package metadata without a version constraint.
* Standardize the package maintainer metadata.
* Contributors: Yan

1.0.1 (2026-08-24)
------------------
* Declare the Elite CS Series SDK dependency.
* Declare geometry messages, standard messages, realtime tools, and example runtime dependencies explicitly.
* Contributors: Ivan

1.0.0 (2026-07-29)
------------------
* First public release of the Elite Robots CS ROS 2 driver.
* Added ros2_control hardware, dashboard, primary, and reverse interfaces.
* Added launch files and controller configurations for Elite CS robots.
* Renamed the package to ``elite_robots_driver``.
* Contributors: Yan
