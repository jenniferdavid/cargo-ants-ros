WARNING: 
THIS file was the readme file on the top of the deprecated GIT repository. 
Information in this file can be erroneous, not updated or deprecated.


Quick setup
===========

This is under construction and likely to change fast. But for now the
idea is to set up a new catkin workspace and use the contents of this
repository as the `src/` subdirectory of that workspace.

    mkdir -p /path/to/workspace
    cd /path/to/workspace
    git clone git@github.com:poftwaresatent/cargo-ants-ros.git src
    cd src
    catkin_init_workspace
    git clone https://github.com/poftwaresatent/estar2.git
    git clone https://github.com/poftwaresatent/sfl2.git
    cd ..
    catkin_make

You should end up with usable ROS nodes etc under `devel/`. If it
fails to build due to message headers not being generated, repeatedly
run `catkin_make` until it works.

3rd Party Components
--------------------

The components related to path planning depend on [E* Version
2][estar2] and [Sunflower Version 2][sfl2] in order to avoid
reinventing the wheel.

[sfl2]: https://github.com/poftwaresatent/sfl2
[estar2]: https://github.com/poftwaresatent/estar2


UDP Bridge for AGVs and ATs
---------------------------

There is example code for this in the cargo_ants_udp package.  At the
time of writing, you can see how a reference trajectory can be
translated from ROS to UDP, passed via UDP to a ROS node which
translates it back to a ROS message and publishes it.

    roscore
    ./devel/lib/cargo_ants_udp/test_node_ros2udp localhost 5555
    rostopic pub -r 1 /testros2udp cargo_ants_msgs/RefenceTrajectory '{start: {secs: 1, nsecs: 10}, dt: 0.1, points: [{ xx: 10, yy: 20, th: 30, xd: 40, yd: 50, thd: 60, xdd: 70, ydd: 80, thdd: 90}] }'

...at this point you should see messages from the ros2udp node...

    rostopic echo /testudp2ros
    ./devel/lib/cargo_ants_udp/test_node_udp2ros 5555

...you should see messages from the udp2ros about data passing through
from UDP to ROS, and messages in the rostopic echo that match the
message you sent with the rostopic pub.


Simulator
---------

The Nepumuk simulator comes bundled with [Sunflower][sfl2].  For
Cargo-ANTs, we rely on version 2 of the simulator, as it supports
picking up and placing down objects.

In the simulator, press SPACE for step-by-step mode, 'c' for
continuous simulation, and 'q' to quit.

Running the Cargo-ANTs mockup entails the following (subject to
change, please check the commit log if these instructions do not
work).

    roscore
    ./devel/lib/cargo_ants_task_scheduler/draft_scheduler
    NPM2PATH=devel/lib ./devel/lib/sfl2/nepumuk2 src/cargo_ants_npm2/config/pickplace-task-2agv-4cnt.yaml

*Notes:* The above assumes that `NPM2PATH=devel/lib` so either set
that on the nepumuk2 command line or in your environment.  There also
are simulator setup files for 1 AGV and 4 containers, 3 AGVs and 6
containers, and maybe more or others by the time you read this. **If you
stop the simulator or the scheduler, you have to restart the other one
as well due to state dependencies between them.**

If you want to get a glimpse of the messages that are involved in the
above, do the following:

    rostopic echo /vehicle_info
    rostopic echo /container_info
    rostopic echo /task

Updating Instructions
---------------------

For the full setup, you need to separately update estar2 and sfl2 (for now at least).

    cd src/estar2
    git pull
    cd ../sfl2
    git pull
    cd ../..
    catkin_make

Outdated Simulator Setups (for future refactoring reference)
------------------------------------------------------------

*Path Adaptor Test*

    roscore
    ./devel/lib/cargo_ants_path_adaptor/path_adaptor alice bob
    NPM_PLUGIN_PATH=devel/lib ./devel/lib/sfl2/nepumuk -c src/cargo_ants_npm/config/cargo-ants.yaml
    rostopic pub -1 bob/path cargo_ants_msgs/Path '{path_id: 1, goals: [{gx: 40, gy: -5, gth: -1.5, dr: 1, dth: 0.1}, {gx: 40, gy: 30, gth: -0.7, dr: 0.1, dth: 0.1}]}'
    rostopic pub -1 alice/path cargo_ants_msgs/Path '{path_id: 1, goals: [{gx: -5, gy: 30, gth: 2, dr: 1, dth: 0.1}, {gx: 35, gy: 18, gth: -1, dr: 0.1, dth: 0.1}]}'

Notice that `NPM_PLUGIN_PATH` needs to point to the directory which
contains the `libcargo-ants-plugin.so` file (file name subject to
change though).  If the rostopic command fails to find the
cargo_ants_msgs/Path type, then you need to source the setup.bash (or
other -- depending on your shell) underneath the devel directory in
the cargo-ants-ros workspace.

*Path Planner (and Path Adaptor) Test*

    roscore
    ./devel/lib/cargo_ants_path_planner/path_planner alice bob
    ./devel/lib/cargo_ants_path_adaptor/path_adaptor alice bob
    NPM_PLUGIN_PATH=devel/lib ./devel/lib/sfl2/nepumuk -c src/cargo_ants_npm/config/cargo-ants.yaml
    rostopic pub -1 /task cargo_ants_msgs/Task '{task_id: 1, vehicle: alice, goals: [{gx: 0, gy: 40, gth: -1.5, dr: 1, dth: 0.1}, {gx: 15, gy: -5, gth: 0, dr: 1, dth: -1}, {gx: 40, gy: 20, gth: -0.7, dr: 1, dth: 0.1}]}'
    rostopic pub -1 /task cargo_ants_msgs/Task '{task_id: 1, vehicle: bob, goals: [{gx: 20, gy: -6, gth: 0, dr: 1, dth: -1}, {gx: 30, gy: -5, gth: 0, dr: 1, dth: -1}, {gx: 40, gy: -6, gth: 0, dr: 1, dth: -1}, {gx: 45, gy: 5, gth: 0, dr: 1, dth: -1}, {gx: 40, gy: 15, gth: 0, dr: 1, dth: -1}]}'

Again, notice the `NPM_PLUGIN_PATH` environment variable.
