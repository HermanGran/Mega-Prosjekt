# ROS Mega Prosjekt

## Hvordan installere third_party repos

1. cd inn til SRC folder
2. Skriv følgende inn i terminalen:
```
vcs import < ../external.repos
```
3. Det vil installere UR config files som trengs for å bruke roboten
4. deretter kjør følgende i mega_ws/ (ikke i src)
```
rosdep install --from-paths src --ignore-src -r -y
```






[![Package nodes diagram simulated on ROS and Rviz | Download Scientific ...](https://tse1.mm.bing.net/th?id=OIP.fs4oCKt3UvUHo7-Q3t0URQHaGJ\&cb=iwp1\&pid=Api)](https://www.researchgate.net/figure/Package-nodes-diagram-simulated-on-ROS-and-Rviz_fig6_316945804)

Based on the project description in the [mega-project.md](https://github.com/adamleon/ais2105/blob/main/project/mega-project.md) file, the task involves programming a UR robot to detect and point to colored cubes placed randomly on a table. The evaluation criteria emphasize the structure and implementation of ROS 2 nodes, camera pipeline robustness, robot movement precision, and configurability.

To address these requirements effectively, a modular ROS 2 architecture with well-defined packages and nodes is recommended. Here's a suggested breakdown:

---

### 📦 Recommended ROS 2 Packages and Nodes

1. **Camera Interface Package**

   * **Node:** `camera_driver_node`

     * Interfaces with the camera hardware and publishes raw image data.
   * **Node:** `image_preprocessor_node`

     * Performs image preprocessing tasks such as filtering and normalization.

2. **Cube Detection Package**

   * **Node:** `cube_detector_node`

     * Processes images to detect colored cubes and publishes their positions.
   * **Node:** `pose_estimator_node`

     * Estimates the 3D poses of detected cubes relative to the robot's base frame.

3. **Robot Control Package**

   * **Node:** `motion_planner_node`

     * Plans the robot's pointing trajectory based on cube positions.
   * **Node:** `ur_robot_controller_node`

     * Sends joint commands to the UR robot to execute the planned motions.

4. **System Integration Package**

   * **Node:** `task_manager_node`

     * Coordinates the overall task flow: triggers image capture, processes detection results, and commands the robot accordingly.
   * **Launch Files:**

     * Define and manage the startup sequence of all nodes with appropriate parameters and configurations.

---

### ⚙️ Configuration and Parameters

* Utilize ROS 2 parameters and YAML configuration files to allow easy tuning of:

  * Camera settings (e.g., exposure, resolution).
  * Detection thresholds (e.g., color ranges for cube detection).
  * Robot motion parameters (e.g., speed, acceleration).
* Implement launch arguments to switch between different modes (e.g., simulation vs. real hardware).

---

### 🧩 Additional Considerations

* **Modularity:** Ensure each package has a single responsibility to enhance maintainability and scalability.
* **Reusability:** Design nodes to be reusable for similar tasks or future projects.
* **Testing:** Include unit tests and integration tests for critical components to ensure reliability.
* **Documentation:** Provide clear documentation for each package, including usage instructions and parameter descriptions.

---

By structuring the project into these packages and nodes, you align with ROS 2 best practices, facilitate easier debugging and testing, and meet the project's evaluation criteria effectively.
