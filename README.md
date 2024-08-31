# Micromouse
Design a micromouse PCB from scratch and implement maze floodfill algorithms on it!  

## Table of Contents

- [Micromouse](#micromouse)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
  - [Project Workflow](#project-workflow)
  - [Hardware Used](#hardware-used)
  - [Perfboard Testing](#perfboard-testing)
  - [PCB Model](#pcb-model)
  - [CAD Models](#cad-models)
    - [Features](#features)
  - [Floodfill Algorithm Simulation](#floodfill-algorithm-simulation)
  - [Software Used](#software-used)
  - [Future Work](#future-work)
  - [Contributors](#contributors)
  - [Resources](#resources)
  - [Acknowledgements](#acknowledgements)
  - [License](#license)

## About The Project
The aim of this project is to design and assemble a full size micromouse which will use floodfill and other path finding algorithms to solve a maze.    

The dimensions of our full-size micromouse is 11.5 x 10 cm

## Project Workflow  
#### Research
- Learning and understanding Embedded C by referring to the firmware used in [SRA WallE](https://github.com/SRA-VJTI/Wall-E) and [MazeBlaze](https://github.com/SurajSonawane2415/MazeBlaze)
- Understanding what Micromouse is and how maze solving algorithms like floodfill actually work
- Deciding the constraints for our micrmouse and selecting hardware and electronic componenets accordingly. We did a detailed review of existing micromouse designs and features and arrived at our present design.
#### CAD and PCB Design
- Designing a CAD Model on [Onshape.com](https://www.onshape.com/en/). We went through various iterations and design options. After weighing the pro's and cons of a 4 motor design, we settled for a 2 motor design with a suction fan in the centre.
- PCB Design keeping mechanical design in mind during routing. Once the pcb design and routing is finalised, the order for the PCB is placed.
#### Perfboard Testing and PID
- Testing our pcb schematic on a perfboard and recreating the final micromouse design on a perfboard.
- Implementing PID on the perfboard Micromouse to refine the code and understand the implementation of control systems.
#### PCB Soldering and Assembly
- Once the PCB is delivered, we will solder the components onto the PCB and assemble the bot with the 3D-printed mounts
- After this, the testing of the final Micromouse in a 6ft x 6ft maze will begin.


## Hardware Used

|     Components        |          Description          |
| --------------------- | ----------------------------- |
| [ESP32 WROOM-E](https://robu.in/product/espressif-esp32-wroom-32e-16m-128mbit-flash-wifi-bluetooth-module/)           | Microcontroller |
| [VL 5310X](https://robu.in/product/gy-53-vl53l0x-laser-tof-flight-time-ranging-sensor-module-serial-port-pwm-output/)  | ToF sensor4 |
| [DRV8833](https://robu.in/product/drv8833-2-channel-dc-motor-driver-module/) | Motor Driver |
| [N12 600 RPM With Encoders](https://robu.in/product/n12-2-4v-1200rpm-metal-gear-motor-with-encoder/) | Motors |
| [IR Sensors](https://robocraze.com/products/5mm-ir-transmitter-receiver-pair?variant=40192346980505&currency=INR&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&campaignid=21586511453&adgroupid=&keyword=&device=c&gad_source=1&gclid=Cj0KCQjw_sq2BhCUARIsAIVqmQtLaP4tnieC83a3KD7llLIWLyPyhrdU7XTjEk-4mGGGtPHHxCPh_YoaAsz_EALw_wcB) | Infrared Emitter-Receiver Pair |


## Perfboard Testing
| ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/base_1.jpeg)            | ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/top_view.jpeg) | ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/perfboard_soldering.jpeg) | ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/perfboard_assembled.jpeg) |
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:

## PCB Model
- View of the PCB Model front and back:       
  
| ![pcb_model_front](https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse_3D_view_f.png)            | ![pcb_model_back](https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse_3D_view_b.png) |    
:-------------------------:|:-------------------------:|

- View of the PCB Routing - front and back:      
  
| ![pcb_routing_front](https://github.com/Ojasp21/Micromouse/blob/main/assets/routing_f.png)            | ![pcb_routing_back](https://github.com/Ojasp21/Micromouse/blob/main/assets/routing_b.png) |  
:-------------------------:|:-------------------------:|

## CAD Models 
- First design iteration was a four motor design with a suction fan. However this design was not suitable for fitting an enitre Esp-Devkit. 

| ![four_motor_design](https://github.com/Ojasp21/Micromouse/blob/main/assets/four_motor_micromouse_CAD.png)            | ![two_motor_design](https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse_assembly.png) |  
:-------------------------:|:-------------------------:|

- Second and final design is a two-motor design with a suction fan. Certain drawbacks of the four-motor design was addressed in this design.  
<p align="center">
  <img src="https://github.com/Ojasp21/Micromouse/blob/main/assets/micromouse%20assembly.gif" width=70% height=70%>
</p>

### Features 
- Suction Fan to improve traction and potentially turn at faster speeds without flipping over.  
- Custom mount to hold the motors, shaft and suction fan in place and reduce the distance between the plate and the ground.
-  Gears to run 4 wheels on two motors  
  
| ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/impeller_design.png)            | ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/custom_mounts.png) | ![](https://github.com/Ojasp21/Micromouse/blob/main/assets/grear_setup.png) |
:-------------------------:|:-------------------------:|:-------------------------:|
  
## Floodfill Algorithm Simulation
The path solving algorithm floodfill has been successfully implemented in the software simulation.  
[Floodfill simulation](https://github.com/Ojasp21/Micromouse/blob/main/assets/flood%20fill%20simulation.mov)

## Software Used 
- Mechanical Design - [Onshape](https://www.onshape.com/en/)
- PCB Design - [KiCad 8.0.4](https://www.kicad.org/)
- Embedded C and Firmware - [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)


## Future Work
- [ ] Implementing the flood-fill algorithm on it
- [ ] Optimizing the bot to achieve high speeds and taking turns as diagonals

## Contributors
- [Ojas Patil](https://github.com/Ojasp21)
- [Varun Nigudkar](https://github.com/fluffysunfish)
- [Vishal Mutha ](https://github.com/Vishal-Mutha)

## Resources
- [UCLA Micromouse Playlist](https://www.youtube.com/playlist?list=PLAWsHzw_h0iiPIaGyXAr44G0XfHfyjOe7) 
- [Mushak Github Repository](https://github.com/gautam-dev-maker/mushak/tree/main)


## Acknowledgements 
- [SRA VJTI](https://sravjti.in/) Eklavya 2024
- Special thanks to our mentors [Atharva Atre](https://github.com/AtharvaAtre), [Suraj Sonwane](https://github.com/SurajSonawane2415), and all the seniors at SRA, VJTI for their constant support and guidance throughout the project.

## License
[MIT License](https://opensource.org/license/mit/)
