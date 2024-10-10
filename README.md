# HACO Blockchain

**Major requirements**

Ubuntu 22.04 LTS (Desktop Version), ROS Humble, IsaacSim, Ganache

**Other requirements**
pip install -r requirements.txt


**Simulation Repo**

https://github.com/abizovnuralem/go2_omniverse

**SDK Repo**

https://github.com/abizovnuralem/go2_ros2_sdk

**Instructions**

Run the simulation repo with environments provided. Set the USD file and number of robots in omniverse_sim.py and run_sim.sh files.
Run the launch file as instructed in SDK repo. 
Then launch Ganache. Set three wallet adresses inside the robot_controller.py 
Then run the following scripts according to the following order.

1) robot_controller.py
2) map_merger.py
3) path_planner.py



