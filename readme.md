 Project Overview

This project implements a Micromouse Maze Solver Bot capable of autonomously exploring and solving mazes using a Costmap-based path planning algorithm. The bot mimics human-like decision-making by exploring unknown paths, building an internal map, and then executing an optimized run to reach the exit in minimal time.

 Hardware Specifications

Microcontroller: ESP32 (dual-core MCU + Wi-Fi, main controller)

Motors: N20 Micro Gear Motors with Encoders (precise motion + odometry)

Motor Driver: TB6612FNG Dual H-Bridge

buck converter: MP1584



Sensors:

TOF (VL53L0X) → Wall detection

Wheels: 2 × 3pi MiniQ wheels + 1 castor wheel (stability)

Battery: 2S Li-ion (7.4V nominal)

Orientation sensor:BNO055

Power Regulation: MP2338 Buck Converters (7.4V → 5V/3.3V rails)

 Software Specifications

Language: C

Algorithm: Costmap System (enhanced Flood Fill)

Simulator Used: Makorone Micromouse Simulator

 Code Structure
File	Purpose
main.c	Central control loop (exploration + optimized run)
solver.c/h	Path planning, BFS costmap, navigation logic
API.c/h	Interface layer for sensors & actuators
queue.c/h	Queue implementation (FIFO) for BFS
 Algorithm – Costmap System

The Costmap system assigns cost values to each maze cell to guide navigation.

Exploration Run

Robot explores step by step using TOF sensors.

Updates wall information → updateMaze().

Expands BFS wavefront from goal cells → updateCosts().

Chooses next move greedily → costmapNavigate().

Optimized Run

Reconstructs shortest path → buildPathByBacktracking().

Executes precomputed path at higher speed.

Supports dynamic replanning if new walls are detected.

 Results: Smoother paths, fewer turns, and faster exit times compared to classic Flood Fill.

 Simulation & Results

Exploration Run Distance: ~509 units

Optimized Path Distance: ~54 units (~89% reduction)

Execution: Consistently reaches exit under competition time limits

 Applications

Autonomous navigation (warehouse robots, delivery bots)

Search & rescue in structured environments

Robotics education & competitions