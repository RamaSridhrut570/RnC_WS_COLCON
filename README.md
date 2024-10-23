# RnC_WS_COLCON


## Open four or required number of terminals then:
## Terminal 1
```sh
colcon build
```
## Every Terminal 
```sh
source install/setup.bash
```


# Task 1B

## To Launch the gazebo-


```sh
ros2 launch eyantra_warehouse task1b.launch.py
```
## To launch moveit

```sh
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py 
```

## To launch script
```sh
python3 src/pymoveit2/examples/task1b_boiler_plate.py
```


# Task 1C

## To Launch the gazebo

```sh
ros2 launch eyantra_warehouse task1c.launch.py  
```

## To launch moveit
```sh
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py 
```

## To launch script
```sh
python3 src/pymoveit2/examples/task1c.py
```


# Task 2a

## To Launch the gazebo

```sh
ros2 launch eyantra_warehouse task2a.launch.py  
```

## To launch moveit
```sh
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py 
```

## To launch script

### To launch publisher
```sh
python3 src/pymoveit2/examples/task1b_boiler_plate.py
```
### To launch main script (listener + Arm Execution)
```sh
python3 src/pymoveit2/examples/task2a.py
```
