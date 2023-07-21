# F1TENTH gym environment ROS2 communication bridge
This is a ROS communication bridge with multiagent support (up to 16) for the F1TENTH gym environment that turns it into a simulation in ROS2. Currently, if more than 4 agents are spawned, performance issues occur.



# Installation

**Supported System:**

- Ubuntu (tested on 20.04) native with ROS 2

## Native on Ubuntu 20.04

**Install the following dependencies:**
- **ROS 2** Follow the instructions [here](https://docs.ros.org/en/foxy/Installation.html) to install ROS 2 Foxy.
- **F1TENTH Gym**
  ```bash
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
  ```

**Installing the simulation:**
- Create a workspace: ```cd $HOME && mkdir -p multiagent_sim_ws/src```
- Clone the repo into the workspace:
  ```bash
  cd $HOME/multiagent_sim_ws/src
  git clone -b multiagent_foxy https://github.com/ibrahimsel/f1tenth_multiagent_gym_ros.git
  ```
- Update correct parameter for path to map file:
  Go to `sim.yaml` [<your_home_dir>/multiagent_sim_ws/src/f1tenth_gym_ros/config](https://github.com/ibrahimsel/f1tenth_multiagent_gym_ros/blob/multiagent_foxy/config/sim.yaml) in your cloned repo, change the `map_path` parameter to point to the correct location. It should be `'<your_home_dir>/multiagent_sim_ws/src/f1tenth_gym_ros/maps/levine'`

- Install dependencies with rosdep then build:
  ```bash
  source /opt/ros/foxy/setup.bash
  cd $HOME/multiagent_sim_ws && rosdep install -i --from-path src --rosdistro foxy -y && colcon build
  ```

# Important things you need to know before use:
## 1. Topics published by the simulation

- `/scan{i}`: The laser scan data for a specific agent. If you were to use the scan data of the 3rd agent, you would subscribe to `/scan3`

- `/car{i}/odom`: The odometry of a specific agent. If you were to use the odometry data of the 4th agent, you would subscribe to `/car4/odom`

- `/map`: The map of the environment

- A `tf` tree is also maintained. If you have tf2_tools installed, you could view the tree with:
```bash
source /opt/ros/foxy/setup.bash
ros2 run tf2_tools view_frames.py
```
Above command is going to give you a .pdf file in the directory you've just executed the command.

## 2. Topics subscribed by the simulation

 `/drive`: The drive topic of all agents (for now) via `AckermannDriveStamped` messages
> Currently, you need to send all of your drive commands for each agent to the `/drive` topic. For sending seperate drive commands to agents, you need to send an `AckermannDriveStamped` message with a `frame_id` of `car{i}/base_link` where {i} is the number of the specific agent you want to choose. 

An example drive message you could send to `/drive` would look like this: 
```bash
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'car1/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```
Above command sends an AckermannDriveStamped message to the first agent.

`/initalpose`: This is the topic for resetting all of the agents' poses to  their initial state via RViz's 2D Pose Estimate tool. There will be a cleverer way to reset agent poses in the future. Do **NOT** publish directly to this topic unless you know what you're doing.


# Launching the Simulation

1. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the terminal:
```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
An rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

You can then run another node by creating another bash session

# Configuring the simulation
- The configuration file for the simulation is at `$HOME/multiagent_sim_ws/src/f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave unchanged.
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file in the container. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name
- The `num_agent` parameter can be changed arbitrarily.
The entire directory of the repo is mounted to a workspace `/multiagent_sim_ws/src` as a package. After changing the configuration, run `colcon build` again in the container workspace to make sure the changes are reflected.

