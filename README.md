# melfa-robot-setup

**Official Repository:**
You can find the official repository for MELFA robots [here](https://github.com/Mitsubishi-Electric-Asia/melfa_ros2_driver.git). The purpose of this repository is to guide you on how to set up the robot in the absence of the Windows configuration software.

## 1. Network Setup

- Create a network on your system with the same subnet as the robot.
- The default IP address of the robot is `192.168.0.20`.
- Set your system's IP address to `192.168.0.21`.

## 2. Steps to Configure the Robot

1. Put the robot in **MANUAL** mode.
2. Ensure that the **TB ENABLE** button is turned **ON** before you start
3. Press the green **EXE** button to open the menu and select **FILE/EDIT**.
4. Create a new script by clicking on **F3**, name the script **SERVER** and enter the following code line by line:

    ```
    1 OPEN "ENET:192.168.0.21" AS #1
    2 MOV P1
    3 MXT1,1,50
    4 MOV P1
    5 HLT
    ```

5. Save this script.
6. Change the robot to **AUTO** mode.
7. Go to the main menu again and select **RUN**, then **OPERATION**. If the file hasn't been chosen, enter the name of the file you saved above.
8. Before starting the script, make sure the robot **SERVO** is turned **ON** (optionally, you can turn **OFF** the TB ENABLE button).
9. Start the program.

## 3. Ready to Run

Once the program is running on the teach pendant and the network is set up on your system, you are ready to start running the robot.

### ROS2 Workspace Setup & Running the Robot

1. **Clone the repository** (with submodules) inside the `src` folder of your ROS2 workspace:
   ```bash
   git clone --recursive https://github.com/harrisonseby/melfa-robot-setup.git
   ```

2. **Install dependencies** from the root of your ROS2 workspace:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build and source the workspace**:
   ```bash
   colcon build && source install/setup.bash
   ```

4. **Launch the robot example**:
   ```bash
   ros2 launch melfa_examples melfa_start.launch.py
   ```
   This will start the robot motion looping between three waypoints.

---

## Viewing/Changing Robot Network Parameters

If you want to view or change the IP address, gateway, netmask, or other parameters of the robot:

1. Head to the main menu by pressing the **EXE** button.
2. Select the **PARAM.** option.
3. Under **NAME**, enter the name of the parameter you wish to view or change.
4. The full list of parameter names is available in the PDF documentation under section 2.2: [Parameter setting](/docs/BFPA3379.pdf)
