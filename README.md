# *DOCUMENTATION*
Documentation for presentation of [EKLAVYA-SRA](https://eklavya.sravjti.in/projects/) project - [MICROMOUSE](https://eklavya.sravjti.in/projects/Micromouse/) 

## **What is Micromouse?**  
Micromouse is a small, autonomous robot designed to navigate and solve a maze. It is a popular robotics competition where participants build and program a robot (“mouse”) to autonomously find its way from a starting point to the center of a maze in the shortest possible time. The competition combines aspects of ***robotics, algorithm development, and engineering design***, making it a challenging and educational project for students, hobbyists, and professionals alike.  

----

### **Key Objectives of a Micromouse Competition:**  
The primary goal in a Micromouse competition is to design a robot that can:  
- **Autonomously Navigate:** The Micromouse must move through a maze without human intervention. It relies on onboard sensors, microprocessors, and algorithms to make decisions in real-time.  
  
-	**Efficiently Solve the Maze:** The robot must find the shortest or fastest route to the goal, usually the center of the maze, from a starting corner. This involves exploring unknown paths, detecting dead-ends, and backtracking when necessary.  

-	**Optimize Time and Path:** The robot’s success is typically measured by the time it takes to reach the goal and the efficiency of the path taken. Robots are expected to minimize turns, backtracks, and wall collisions to achieve the fastest time.

### **Core Components of a Micromouse:**  
1.	**Microcontroller**: Acts as the “brain” of the robot, processing inputs from sensors and executing maze-solving algorithms. Common microcontrollers used are Arduino, ESP32, or STM32.

2.	**Sensors**: Typically, Infrared (IR) sensors or Time-of-Flight (ToF) sensors are used to detect walls and measure distances, helping the robot navigate through the maze accurately.

3.	**Motors and Motor Drivers**: The Micromouse uses small DC motors or stepper motors for movement, controlled by motor drivers like the DRV8833, which regulate speed and direction.

4.	**Algorithms**: Efficient maze-solving algorithms, such as Flood-Fill, Wall-Following, A*, or Dijkstra’s algorithm, are implemented to ensure the mouse navigates optimally.

5.	**Power Supply**: A compact power source, like a 3.7V lithium-ion battery, is used to power all components while keeping the weight and size minimal.

## **Educational and Practical Value:- Domains explored**   
The Micromouse competition offers significant learning opportunities in embedded systems, control theory, robotics, artificial intelligence, and mechanical design. It encourages participants to innovate, optimize algorithms, and build practical skills in programming and electronics. The competition’s real-world challenges, such as sensor noise, motor control, and algorithm efficiency, make it an excellent platform for learning and experimentation.  

----
### **Maze solving algorithms:**  
Maze-solving algorithms are crucial for enabling a Micromouse robot to navigate through a maze efficiently and reach a designated goal, typically the maze’s center. These algorithms determine the path by processing real-time data from sensors and making decisions based on the maze’s layout.Some of the most popular algorithms include the **Flood-Fill algorithm**, which dynamically updates a grid map by assigning values to each cell, allowing the robot to always move toward the cell with the lowest value, thus finding the shortest path to the goal.  
Another common approach is the **Wall-Following algorithm**, such as the Left or Right Wall Following, which directs the robot to follow one side of the maze walls until it reaches the goal. More advanced algorithms, like **A**\* and **Dijkstra’s algorithm**, calculates the most efficient path by considering both the distance and cost associated with each possible move, ensuring optimal navigation. The choice of algorithm depends on the maze’s complexity and the desired balance between speed, accuracy, and computational efficiency.

## **Flood-Fill Algorithm:**  
The Flood-Fill algorithm is a systematic way of exploring the maze, updating the shortest path dynamically as new information about the maze is discovered.

![FloodFill_CELL_VALUES](https://github.com/Ojasp21/Micromouse/blob/main/assets/FLOODFILLL.webp)

### **Working Principle:**  
1.	**Initialization**:  
    - The goal cell (usually the center of the maze) is set to 0\.  
	- All other cells are initialized with a high value, indicating a “flooded” or unvisited state. 
   
2.	**Updating Cell Values**:  
	- Starting from the goal cell, each adjacent cell is assigned a value one greater than the current cell.  
	- This process continues until all reachable cells are assigned values representing the number of steps required to reach the goal.
  
3.	**Mouse Movement**:  
	- The mouse always moves to the adjacent cell with the lowest value. This ensures it is moving toward the goal.  
	- As the mouse explores and detects walls, the values are recalculated to account for these new obstacles. Cells adjacent to walls have their values increased.
  
4. **Handling Dead-Ends and Backtracking**:  
    - If a dead-end is reached, the mouse will backtrack to the nearest cell with an alternative path. The algorithm dynamically updates the flood values to reflect the most efficient path.

## **Advantages of Flood-Fill:**
- **Optimal Pathfinding**: Guarantees the shortest path by continuously updating the maze’s knowledge.
    
- **Adaptability**: Reacts dynamically to new information about the maze structure.
  
- **Efficiency**: Minimizes unnecessary movements and backtracks, saving time and battery.    
  
---
### **Testing Flood-Fill Algorithm :**  

<p align="center">
  <img src="https://github.com/Ojasp21/Micromouse/blob/main/assets/FLOODFILL_GIF.gif" width=70% height=70%>
</p>

#### To test the Flood-Fill algorithm in a simulation:  
- Set Up Different Maze Configurations: Create varying levels of maze complexity to ensure the algorithm adapts and recalculates paths effectively.  
- Simulate Sensor Accuracy: Use realistic sensor data from the simulation to mimic actual wall detection and noise.  
- Measure Performance Metrics: Track the time taken to reach the goal, the number of recalculations, and the total distance traveled to evaluate algorithm efficiency.  
- Optimize Parameters: Fine-tune the sensor sensitivity, robot speed, and recalculation intervals to improve performance based on the test results.

## **Left Wall Following Algorithm:**  
The Left Wall Following method is a simpler heuristic-based algorithm that instructs the Micromouse to always follow the left wall of the maze. This method is useful for exploring mazes that do not have isolated areas or loops. 

#### **Working Principle:**

1. **Initial Movement**:  
   - The mouse starts from the initial position and moves forward while checking for walls.
  
2. **Always Turn Left**:  
   - When encountering an intersection or corner, the mouse prioritizes turning left if there is no wall.  

   - If a wall is present on the left, it will continue moving forward until a left turn is possible.  

   - If both left and forward paths are blocked, the mouse turns right.  

3. **Handling Dead-Ends**:  
   - When reaching a dead-end, the mouse turns 180 degrees to backtrack and follows the same left-wall rule.  

4. **Reaching the Goal**:  
    - The mouse continues following the left wall until it reaches the center of the maze or the designated goal.

## **Advantages of Left Wall Following:**  
- **Simplicity**: Easy to implement with minimal computational requirements.  
  
- **Predictable**: Provides a straightforward way to navigate simple mazes.  

- **Quick Exploration**: Ideal for initial maze exploration to map walls and potential paths.
-----

#### **Testing Left Wall Following Algorithm:**
To test the Left Wall Following algorithm in a simulation:  

- **Set Up Various Mazes**: Include mazes with no isolated sections, loops, or walls that form a continuous boundary to test the effectiveness of the Left Wall Following method.

- **Simulate Sensor Inputs**: Mimic real-world sensor behavior to detect walls and ensure reliable decision-making for turning.  
  
- **Evaluate Performance**: Measure the time to complete the maze, the number of turns, and the efficiency of the path taken compared to the optimal path.  

- **Identify Limitations**: Test in more complex mazes to understand scenarios where the Left Wall Following method may not yield the shortest path or may get stuck in loops.
-----

# **Start of the project…..**

## **Perfboard Testing for Micromouse:**
Perfboard testing is an essential step in the development process of a Micromouse robot. A perfboard (short for perforated board) is a prototyping board used to build circuits without the need for designing and manufacturing a printed circuit board (PCB) right away. It allows developers to test the functionality of various components, such as sensors, motor drivers, and microcontrollers, before finalizing the design. This approach provides a flexible and cost-effective way to verify that all components work as intended when integrated.

| ![PERFBOARD_IMAGE_1](https://github.com/Ojasp21/Micromouse/blob/main/assets/perfboard_assembled.jpeg) | ![PERFBOARD_IMAGE_2](https://github.com/Ojasp21/Micromouse/blob/main/assets/perfboard_soldering.jpeg) |
:------------------|:-----------------:|

### **Purpose of Perfboard Testing:**  
1. **Circuit Validation**: Ensures that all components are correctly wired and interact as expected. This step is crucial to identify any electrical issues, such as incorrect connections or voltage mismatches, which could lead to component failure.  
2.	**Functional Testing**: Allows testing of the Micromouse’s core functionalities—like motor control, sensor readings, and microcontroller processing—in a controlled environment. This helps in validating that the hardware and software communicate properly.  
3.	**Debugging and Optimization**: Provides a platform for real-time debugging. One can easily replace components, adjust connections, or tweak circuit parameters without the complexities associated with a final PCB layout.

### **Steps for Perfboard Testing:**  
1.  **Component Placement and Soldering**:  
    - Carefully place all components—such as microcontrollers (ESP32), motor drivers (DRV8833), IR sensors, and Time-of-Flight (ToF) sensors—on the perfboard in a logical layout that minimizes wire length and prevents cross-connections.  
	- Solder the components securely, ensuring no cold joints or short circuits between adjacent pads. Proper soldering techniques are crucial to avoid unstable connections.  
2.  **Power Supply Verification**:  
	- Connect the power source (e.g., 3.7V battery) to the perfboard and verify the voltage levels across different sections of the circuit. Use a multimeter to ensure that all components receive their required voltage and current levels.  
3.  **Initial Power-On Testing**:  
	- Perform a “smoke test” by powering on the circuit with minimal components to check for immediate signs of faults, such as overheating components, unusual noises, or visible smoke.  
4.  **Subsystem Testing**:  
	- Motor and Motor Driver Testing: Verify that the motors respond correctly to the control signals from the motor driver and microcontroller. Test different speeds and directions.  
	- Sensor Testing: Confirm that the sensors (IR sensors, VL53L0X ToF sensors) provide accurate readings and that data is correctly processed by the microcontroller.  
	- Microcontroller Functionality: Ensure the microcontroller executes the maze-solving algorithms correctly and interfaces with all other components without errors.  
5.	**Integration Testing**:  
	- Once all individual components and subsystems are tested, perform an integrated test to see how they work together. This step involves running simple maze-solving algorithms, like wall-following, to ensure the robot moves and navigates properly.


### **Benefits of Perfboard Testing:**  
- Flexibility and Easy Modifications: Components can be easily removed or replaced without the need to manufacture a new PCB.  
- Cost-Effective: Helps in identifying issues early in the development cycle, saving costs associated with faulty PCB design or incorrect component selection.  
- Prototyping Speed: Accelerates the prototyping process, allowing for rapid iterations and testing of different circuit designs or configurations.

## **Cad Model:**
| ![four_motor_design](https://github.com/Ojasp21/Micromouse/blob/main/assets/four_motor_micromouse_CAD.png)            | ![two_motor_design](https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse_assembly.png) |  
:-------------------------:|:-------------------------:|

<p align="center">
  <img src="https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse%20assembly.gif" width=70% height=70%>
</p>

## **PCB Designing :**
| ![pcb_model_front](https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse_3D_view_f.png)            | ![pcb_model_back](https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse_3D_view_b.png) |    
:-------------------------:|:-------------------------:|

| ![pcb_routing_front](https://github.com/Ojasp21/Micromouse/blob/main/assets/routing_f.png)            | ![pcb_routing_back](https://github.com/Ojasp21/Micromouse/blob/main/assets/routing_b.png) |  
:-------------------------:|:-------------------------:|

## **What is PCB Design?**  
PCB (Printed Circuit Board) design involves creating a physical layout for a board that supports and electrically connects electronic components. The process includes schematic capture, layout design, component placement, routing, and verification.  

### **Key Components of PCB Design:**  
- **Schematic Design**: The first step in PCB design, where electronic circuits are made using CAD tools to define electrical connections between components.  

- **PCB Layout**: Involves designing the physical layout of the board, defining its shape, and placing components based on the schematic.  

- **Routing**: Establishing electrical paths (traces) that connect various components on the board, which is crucial for signal integrity and reducing noise.  

- **Verification and DFM Checks**: Ensures the design is functionally correct and manufacturable. DFM (Design for Manufacturing) checks help identify potential issues before production.

### **Tools for PCB Design:**  

- **KiCad**: A widely-used open-source PCB design tool that supports schematic capture, PCB layout, and DFM checks.  

- **Other Tools**: Eagle, Altium Designer, and OrCAD are other popular tools used in professional PCB design.

#### **Basic Steps in PCB Design Using KiCad:**  
1. **Create Schematic**: Define components and their connections.  

2. **Assign Footprints**: Link each schematic symbol to its corresponding physical footprint.  

3. **Design Board Layout**: Arrange components and define the board’s shape and size.  

4. **Route Traces**: Connect components following design rules to ensure electrical performance.  

5. **Run DFM Checks**: Verify that the design is free from errors that could cause manufacturing issues.

## **Power and Sensor Circuitry Design for Micromouse :**

### **Introduction to Power and Sensor Circuitry:**  
The Micromouse project involves designing a PCB that integrates multiple components, such as motor drivers, distance sensors, and voltage regulators, to navigate a maze autonomously. Key components include the DRV8833 motor driver, VL53L0X ToF sensor, IR sensors, and the XL4015 voltage regulator.

#### **Using XL4015 in PCB Design:**  

- **XL4015 Overview**: A high-efficiency, step-down (buck) DC-DC converter providing up to 5A output current. It is used in the design to manage power supply for the Micromouse components.  

- **Key Considerations**:  
  
  - **Thermal Management**: Ensure sufficient copper pours and thermal vias to dissipate heat generated by the regulator.  
  
  - **Capacitor and Inductor Selection**: Use low ESR capacitors for stability and select inductors based on current and ripple requirements.  
  
  - **Layout Tips**: Minimize trace lengths for the input and output capacitors to reduce noise. Use a solid ground plane for improved stability.

#### **Integrating DRV8833 Motor Driver:**  

- **DRV8833 Overview**: A dual H-Bridge motor driver capable of driving two DC motors or one stepper motor. It is used for controlling the Micromouse’s motors.  

- **Key Considerations**:  
	
	- **Heat Dissipation**: Ensure adequate heat sinking and thermal vias for cooling the motor driver.  
	
	- **Routing Strategy**: Use wide traces for motor supply lines to handle high current. Keep control signal traces short to avoid noise interference.  
	
	- **Placement**: Position the DRV8833 close to the motors to reduce trace inductance and resistive losses.

#### **Integrating VL53L0X ToF Sensor:**  

- **VL53L0X Overview**: A Time-of-Flight (ToF) sensor used for distance measurement, critical for detecting walls in the maze.  

- **Key Considerations**:  
	
	- **I2C Communication**: Ensure proper pull-up resistors are in place for I2C lines. Keep the traces short to avoid signal degradation.  
	
	- **Component Placement**: Place the sensor on the front side of the Micromouse to maximize its field of view.

#### **Using IR Sensors:**  

- **IR Sensors Overview**: Used for close-range object detection and alignment in the maze.  
	
	- **Key Considerations**: Important for PID tuning of the bot for moving in the straight path.  
    
	- **Analog Signal Integrity**: Ensure clean analog ground planes and minimize noise by separating analog and digital signals.  
	
	- **Placement Strategy**: Position IR sensors around the perimeter of the Micromouse to cover all directions for effective wall-following algorithms.

#### **Using 3.7V battery:**  

- The mouse is powered by  a 3.7V battery, which provides a compact and efficient power source suitable for driving the components like the DRV8833 motor driver and sensors.   

- Proper regulation using the XL4015 ensures stable voltage levels for reliable operation.

### **Resolving DFM Issues that we faced and Tips in KiCad:**
---

#### **Common DFM Issues in PCB Design:**  

- **Component Spacing**: Ensure sufficient spacing between components to allow for effective soldering and heat dissipation.  

- **Trace Width and Spacing**: Validate trace widths based on current-carrying requirements and maintain proper spacing to prevent short circuits.  

- **Silkscreen Overlap**: Avoid placing silkscreen over solder pads to prevent soldering issues.  

- **Via Size and Placement**: Use adequately sized vias to handle required current and minimize parasitic effects.

#### **Addressing DFM Issues in KiCad:**  

- **Design Rule Checks (DRC)**: Run DRC in KiCad to identify and correct design rule violations, such as insufficient trace width or clearance.  

- **Layer Stackup Management**: Properly define the layer stack-up to ensure signal integrity, reduce noise, and provide effective grounding.  

- **Assembly Information**: Provide detailed assembly information, such as component part numbers, orientations, and placement guidelines.  

- **Solder Mask and Paste Layer Checks**: Ensure that the solder mask does not cover solder pads and that the paste layer aligns correctly for effective soldering.

### **Advanced Tips for Optimizing PCB Design :**  

- Thermal Analysis: Conduct thermal simulations to predict heat distribution and optimize the placement of thermal vias and copper pours for better heat dissipation.  

- Signal Integrity Analysis: For high-speed designs, consider impedance matching to avoid signal reflections and degradation.  

- Component Placement Strategy: Optimize placement for better electrical performance and assembly ease. Keep power components like the XL4015 away from sensitive analog circuits.  

- Documentation and Output Files: Generate comprehensive documentation, including Gerber files, drill files, and the Bill of Materials (BOM), for seamless manufacturing.

## **Simulation for Maze solving :**

### **Introduction to Micromouse Maze Solving:**

Micromouse maze-solving is a robotics competition where an autonomous robotic mouse (Micromouse) navigates a maze to reach the center in the shortest time possible. The maze-solving algorithm, combined with accurate sensor data and efficient motor control, plays a critical role in the Micromouse’s success.  

### **Simulation Benefits**:  

Running a simulation for a Micromouse maze-solving project offers several advantages:  

- **Algorithm Testing**: Allows developers to test and refine their maze-solving algorithms without the need for physical hardware.  

- **Error Debugging**: Enables identification of logical errors and optimizes the robot’s decision-making processes.  

- **Cost Efficiency**: Reduces the cost and time associated with repeated physical trials and hardware wear and tear.

### **Simulation Environment Setup:**  

1. **Software Requirements**:  

   - We have used a online simulation for our bot with custom API’s  

   - We have used an online simulator for micromouse and used its environment related API’s.  

2.	**Modeling the Micromouse**:  
	
	- Create or import a 3D model of the Micromouse with accurate dimensions and physical properties.  
    
	- Define sensors (e.g., IR sensors, ToF sensors) and actuators (e.g., motor drivers) in the simulation environment. Ensure that these are correctly parameterized to match the real-world counterparts.  

3.	**Maze Environment Setup**:  
   
    - Design the maze according to competition rules, with defined wall thickness, cell size, and possible obstacles.  
    
	- Set up the maze as an environment file in the simulation software. Include parameters for ground friction, wall material, and any other environmental factors affecting movement.

### **Integration of Maze-Solving Algorithms:**  

- **Algorithm Types**:  
	
	- Implement algorithms such as left wall following and floofill.  
	
	- Optimize algorithms to account for pathfinding efficiency and avoid unnecessary turns or backtracking.  

- **Programming the Robot**:  
	
	- Write code for movement control, sensor data processing, and decision-making based on sensor inputs.  
	
	- Implement an API that allows communication between the simulation environment and the maze-solving code.

## **What’s next..**

1. We will be building a maze with the help of ply and after assembling our pcb, start with the testing.  

2. Rectify the mistakes in our design and will make a new pcb for the next iteration.  

3. Then target the implementation of different algorithms. 

