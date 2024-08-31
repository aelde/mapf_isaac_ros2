# MAPF project
this pj use cbs(astar) for control(planner ,follower) and simulation on isaac sim.

## about this project
how pj show how to plan multi robot paths and control them in ros and simulation on isaac.

## Installation
install [python](https://www.python.org/downloads/),
install [ros-humble](https://docs.ros.org/en/humble/Installation.html) because this project requires it. The project also has custom messages and services, so use **colcon** build to complete the setup.

## Usage
**all maps** can be found in **/mapf_isaac/_map.py(np.array map)**  
each np.array map refferent each node in isaac sim(pic below)  
<img src="/z_img_readme/image.png" alt="drawing" width="600"/>  

to create your custom map create new np.array of node in isaac variable in **/mapf_isaac/_map.py** then use **_map_generate.py** and **_rotate_map.py** to make map like this(use in cbs)  
<img src="/z_img_readme/img2.png" alt="drawing" width="200"/>  

- to config all about env -> config the **/config/uneven_astar.yaml** 
can use up to 10 robots(can custom robot start position in yaml)  

- all isaac sim world env stored in **/isaac/** 

- all cbs and aster logic stored in **/all_planner**  
    - if want to config logic(eg. change animation visualize version) change these is **DICISION.py**
    - cbs with robot state(last updated) is **cbs_basic_with_rot_state.py**  
    - astar with robot state(last updated) is **a_star_class_with_rot_state.py**  

## Run Project

if want to run this pj(ros+isaac) following this step  
- open 4 teminals
    - teminal 1: **ros2 run mapf_isaac ros_run_isaacsim.py**(for open isaac)  
    - teminal 2: **ros2 run mapf_isaac ros_tbJob_service_server.py**(for run service)  
    - teminal 3: **ros2 run mapf_isaac ros_planner_v1.py.py**(for run path planner node)  
    - teminal 4: cd **/mapf_isaac/scripts** and run **python ros_pathfol_robot_state.py**(for run path follower node)  
- and next must sent service call to assign goal of each robots
    - service msg structure like this 
```bash
ros2 service call /tb_job_srv mapf_isaac/srv/TbJob "start_routing: false
tb_id: 1
goal_point:
x: 4.5
y: -46.5
z: 0.0"
```  
**start_routing** : **true** is start planner(if want to plan for multi robot sent false and the last robot sent true)  
**tb_id** : if a specific robot id is given, the system will use that robot. If the ID is -1, it will ignore the robot id and choose the robot closest to the specified position  
**goal_point** : x,y,z

See the result...

## License

[EIEI](https://choosealicense.com/licenses/mit/)