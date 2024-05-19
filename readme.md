Ruthwik Dasyam 120405563
ENPM662 - Planning for Autonomous Robots

workflow : 
- Setting up panda robot for moveit setup assistant
- Creating a new workspace with panda_description and ruth_configuration (from setup assistant)
- creating a new package for cpp script
- determining joint angles for pick place configurations, for arm and gripper
- Once joint angles are given in the code, in order of execution
- Launch Rviz and run the cpp code

No extra libraries used ::

Steps to visualize the script
- move the package 'package_120405563' to your workspace containing panda_description and setup_configuration (from moveit setup assistant) packages
- colon build
- launch rviz from your configuration package
   ros2 launch <configuration_package_name> demo.launch.py
- In other terminal, run the script
ros2 run package_120405563 ruth_script

The panda arm performing pick and place action is visualized in rviz