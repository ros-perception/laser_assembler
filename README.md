This file describes the work done and steps to perform test on the laser_assembler package with crystal release.

"https://github.com/vandanamandlik/laser_assembler/tree/ros2-crystal"


ROS2 Migration changes

	The basic concept and design are same as ROS1.
	Work Done by referring ROS1 hydro-devel branch of laser_assembler package.
	All changes for migration have been done as per Migration guide.
		Migrated all header and source files into ROS2 style
		Migrated yaml files into ROS2 style
		Migrated .launch file into launch.py in ROS2 style
		Migrated CMakeLists.txt and package.xml in ROS2 style

Dependencies

	filters
	laser_geometry
	message_filters
	launch

Build packages

	mkdir test_laser_assembler

	Build filters package
		Go to test_laser_assembler directory
		mkdir filters_ws
		mkdir filters_ws/src
		cd filters_ws/src
		git clone https://github.com/swatifulzele/filters.git -b ros2_devel
		Go to filters_ws directory.
		source /opt/ros/crystal/setup.bash
		colcon build

	Build laser_geometry package (laser_geometry package was not having PointCloud1 support so I have added it here.)
		Go to test_laser_assembler directory
		mkdir laser_geometry_ws
		mkdir laser_geometry_ws/src
		cd laser_geometry_ws/src
		git clone https://github.com/vandanamandlik/laser_geometry.git -b ros2-devel
		Go to laser_geometry_ws directory.
		colcon build

	Build laser_assembler
		Go to test_laser_assembler directory
		mkdir laser_assembler_ws
		mkdir laser_assembler_ws/src
		git clone https://github.com/vandanamandlik/laser_assembler.git -b ros2-crystal
		remove laser_assembler_srv_gen folder from laser_assembler and paste it in src folder (so your src folder should contain laser_assembler and laser_assembler_srv_gen directories.)
		Go to laser_assembler_ws
		source <filters's setup.bash file path> (eg. source filters_ws/install/setup.sh)
		source <laser_geometry's setup.bash file path>
		colcon build

Do the test

	Here launch files are used independently.
	Following are the steps to run the test cases independently:

	1.  Set the path
		Go to laser_assembler_ws
		source /opt/ros/crystal/setup.bash
		source ./install/setup.bash
 
	2. To run non_zero_point_cloud_test
		ros2 launch laser_assembler test_laser_assembler.launch.py

Limitations

	colcon test does not work as launch.py files can not be executed/added with CMakeLists.txt as of now.

