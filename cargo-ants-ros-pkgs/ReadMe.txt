
This file summarizes all the required steps in order to correctly set-up a computer to run Cargo-ANTs WP4 and WP5 software.

(I) SYSTEM 
    - Ubuntu 14.04 LTS [ see intructions at www.ubuntu.com ]
    
(II) THIRD-PARTY PACKAGES AND LIBRARIES
    - C++ compiler with C++11 support, v4.8.2 or + 
            sudo apt-get install build-essential
    - Cmake
            sudo apt-get install cmake
    - ROS Indigo. Desktop-Full.
            Please see intstructions at wiki.ros.org/indigo/Installation/Ubuntu
    - Eigen 3.2 
            Already installed with ROS
    - OpenCV 2.4 
            Already installed with ROS
    - Boost 
            Already installed with ROS
    - Ceres 1.10.
            Detailed 5-step install procedure at http://wiki.iri.upc.edu/index.php/Wolf
    - OpenGL, GLU, GLUT
            sudo apt-get install freeglut3-dev
    - GTK+ 
            sudo apt-get install ?? (TODO: Define better which is the required ubuntu package)
            
(III) CARGO-ANTS ROS PACKAGES (at TNO SVN)
    - Move to your ros-workspace/src
            1) svn checkout https://e-svn.tno.nl:18080/svn/cargo_ants.repo/code/cargo-ants-ros-pkgs
            2) cd ..
            3) catkin_make  //builds the whole workspace
            4) catkin_make --only-pkg-with-deps [pkg_name]  //builds only a specified package
            
(IV) CSIC-UPC SW
    - iri_core 
            1) roscd //move to your ros-workspace/devel
            2) cd ../src
            3) svn checkout https://devel.iri.upc.edu/pub/labrobotica/ros/iri-ros-pkg_hydro/metapackages/iri_core
            4) //this package will be compiled later as a dependency of several IRI ros packages. 
    - Laser Scan Utils 
            1) svn checkout https://devel.iri.upc.edu/pub/labrobotica/algorithms/laser_scan_utils/tags/v1 laser_scan_utils
            2) cd laser_scan_utils/build
            3) cmake ..
            4) make
            5) sudo make install
    - Local Mapping
	    1) // Local mapping is in cargo-ants-ros-pkgs. See section (III) above to find checkout instructions of this repository.
	    2) catkin_make --only-pkg-with-deps cargo_ants_local_mapping
    - Wolf (SLAM)
            1) svn checkout https://devel.iri.upc.edu/pub/labrobotica/algorithms/wolf/tags/v1 wolf
            2) cd wolf/build
            3) cmake ..
            4) make
            5) sudo make install
            6) roscd //move to your ros-workspace
            7) cd ..
            8) svn checkout https://devel.iri.upc.edu/pub/labrobotica/ros/tags/iri_wolf/v1/ src/iri_wolf
            9) catkin_make --only-pkg-with-deps iri_wolf
            
(V) HH SW
    - E* 
            1) git clone https://github.com/poftwaresatent/estar2
            2) mkdir build
            3) cd build
            4) cmake ..
            5) make
            6) sudo make install
    - Sunflower 
            1) git clone https://github.com/poftwaresatent/sfl2
            2) mkdir build
            3) cd build
            4) cmake ..
            5) make
            6) sudo make install
    - Chomp
            1) git clone https://github.com/poftwaresatent/trychomp
            2) mkdir build
            3) cd build
            4) cmake ..
            5) make
            6) sudo make install

(VI) SIMULATION ENVIRONMENT
	1) Assuming ros-indigo-desktop-full is installed. See section (II).
	2) Install required extra packages
		sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers ros-indigo-gazebo-ros-control
    3) Install required extra packages
        sudo apt-get install ros-indigo-teleop-twist-keyboard
	4) All sources are at cargo-ants-ros-pkgs. See section (III) above to find  checkout instructions of this repository.
	5) Edit your /usr/share/gazebo/setup.sh file (assuming cargo-ants-models checked out at my_dev_path folder): 
		- Add the following line: export GAZEBO_MODEL_PATH=my_dev_path/cargo-ants-models
		- Add the following paths to the GAZEBO_RESOURCE_PATH environment variable, separated by ":"
			 my_path_to_ros_workspace/src/cargo-ants-ros-pkgs/agv_gazebo
			 my_path_to_ros_workspace/src/cargo-ants-ros-pkgs/at_gazebo
	6) Add the following line to your ~/.bashrc file: 
		source /usr/share/gazebo/setup.sh
	7) build platform drive_sim packages
		catkin_make --only-pkg-with-deps agv_drive_sim
	8) execute a test:
		roslaunch agv_tests_sim agv_sim_01_drive.launch rviz:=True


