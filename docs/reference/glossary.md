---
sidebar_position: 1
---

# Glossary

## Key Concepts in Physical AI and Humanoid Robotics

---

### A

**Action (ROS 2)**  
A communication pattern in ROS 2 for long-running, goal-oriented tasks that provide continuous feedback and can be canceled. Used for complex behaviors like navigation or manipulation.

**Actuator**  
A mechanical device that converts energy (electrical, hydraulic, pneumatic) into motion. In robotics, actuators control joint movements and end-effector actions.

---

### B

**Bipedal Locomotion**  
Walking or running on two legs. Requires sophisticated control to maintain balance and stability, making it one of the most challenging aspects of humanoid robotics.

---

### C

**Center of Mass (CoM)**  
The average position of all mass in a system. For humanoid robots, controlling the CoM is critical for maintaining balance during locomotion and manipulation.

**Cognitive Robotics**  
The field of robotics focused on developing robots with cognitive capabilities such as reasoning, learning, planning, and natural interaction with humans.

**Costmap**  
A grid-based representation of the environment used in navigation, where each cell contains information about obstacles and traversability.

---

### D

**Digital Twin**  
A virtual replica of a physical system that mirrors its state, behavior, and environment. In robotics, digital twins enable safe testing and rapid iteration in simulation.

**Domain Randomization**  
A technique for improving sim-to-real transfer by training AI models on simulated environments with randomized parameters (lighting, textures, physics).

**Degrees of Freedom (DOF)**  
The number of independent parameters that define the configuration of a mechanical system. A humanoid robot typically has 20-30 DOF.

---

### E

**Embodied Intelligence**  
The concept that an intelligent agent's cognitive capabilities are deeply intertwined with its physical body and interactions with the environment.

**End-Effector**  
The device at the end of a robotic arm or manipulator that interacts with the environment (e.g., gripper, hand, tool).

---

### F

**Forward Kinematics**  
The process of calculating the position and orientation of a robot's end-effector given its joint angles.

---

### G

**Gait**  
A rhythmic pattern of limb movements used for locomotion. Common gaits include walking, running, and hopping.

**Gazebo**  
An open-source 3D robotics simulator with accurate physics simulation and extensive ROS integration.

**GPT (Generative Pre-trained Transformer)**  
A large language model architecture used for natural language understanding, generation, and task planning in conversational robotics.

---

### H

**Humanoid Dynamics**  
The study of forces and motions governing humanoid robots, including balance, stability, and whole-body coordination.

---

### I

**IMU (Inertial Measurement Unit)**  
A sensor that measures orientation, angular velocity, and linear acceleration using accelerometers and gyroscopes.

**Inverse Kinematics**  
The process of calculating the joint angles required to place a robot's end-effector at a desired position and orientation.

**Isaac Sim**  
NVIDIA's photorealistic robotics simulation platform built on Omniverse, designed for AI training and synthetic data generation.

---

### J

**Joint**  
A connection between two links in a robot that allows relative motion. Common types include revolute (rotational) and prismatic (linear).

---

### K

**Kinematics**  
The study of motion without considering forces. In robotics, kinematics describes the relationship between joint angles and end-effector positions.

---

### L

**LiDAR (Light Detection and Ranging)**  
A sensor that measures distances by illuminating targets with laser light and measuring the reflection with a sensor.

**Localization**  
The process of determining a robot's position and orientation within an environment.

---

### M

**Manipulation**  
The act of physically interacting with objects in the environment, typically using robotic arms or grippers.

**Mapping**  
The process of building a representation of an environment, often as a 2D occupancy grid or 3D point cloud.

**Model Predictive Control (MPC)**  
A control strategy that optimizes a sequence of future control actions by predicting system behavior over a finite time horizon.

**Multimodal Fusion**  
Combining information from multiple sensor modalities (vision, audio, touch) to improve perception and decision-making.

---

### N

**Nav2 (Navigation 2)**  
The second generation of ROS navigation software, providing flexible path planning and control for mobile robots.

**Node (ROS 2)**  
An executable process in ROS 2 that performs a specific task and communicates with other nodes via topics, services, or actions.

---

### O

**Odometry**  
The estimation of a robot's position and orientation over time based on motion sensor data (wheel encoders, IMU, visual features).

---

### P

**Perception**  
The process by which robots acquire, interpret, and understand information from their environment using sensors.

**Physical AI**  
The study and development of intelligent systems that perceive, reason, and act within physical environments, as opposed to purely digital AI.

**PhysX**  
NVIDIA's physics simulation engine used in Isaac Sim and other robotics platforms for accurate rigid body dynamics.

---

### R

**RGB-D Camera**  
A camera that captures both color (RGB) and depth (D) information, enabling 3D perception.

**ROS 2 (Robot Operating System 2)**  
A flexible framework for writing robot software, providing tools and libraries for building distributed robotic systems.

**Reinforcement Learning (RL)**  
A machine learning paradigm where an agent learns to make decisions by interacting with an environment and receiving rewards.

---

### S

**Service (ROS 2)**  
A synchronous request-response communication pattern in ROS 2, used for one-time operations like configuration or queries.

**SLAM (Simultaneous Localization and Mapping)**  
A computational problem where a robot simultaneously constructs a map of an unknown environment while localizing itself within that map.

**Synthetic Data**  
Artificially generated data created in simulation, used to train AI models without requiring real-world data collection.

---

### T

**Topic (ROS 2)**  
An asynchronous publish-subscribe communication channel in ROS 2, used for continuous data streams like sensor readings.

---

### U

**URDF (Unified Robot Description Format)**  
An XML format for describing robot kinematics, dynamics, and visual properties in ROS.

**Unity**  
A real-time 3D development platform used for high-fidelity robot visualization and human-robot interaction studies.

---

### V

**VLA (Vision-Language-Action)**  
A system architecture that integrates visual perception, natural language understanding, and physical action generation for cognitive robotics.

**VSLAM (Visual SLAM)**  
SLAM using camera images as the primary sensor input, enabling localization and mapping through visual features.

---

### W

**Whisper**  
OpenAI's automatic speech recognition (ASR) system, used for converting human speech to text in voice-controlled robotics.

**Whole-Body Control**  
A control strategy that coordinates all joints of a humanoid robot simultaneously to achieve desired movements while maintaining balance.

---

### X

**XACRO (XML Macros)**  
An extension of URDF that adds programmable features like macros, properties, and mathematical expressions for more maintainable robot descriptions.

---

### Z

**Zero Moment Point (ZMP)**  
A point on the ground where the total moment from gravity and inertia forces equals zero. Critical for bipedal balance control.

---

**Navigation:**  
← [Hardware Architecture](../hardware-architecture/overview.md) | [Bibliography →](./bibliography.md)
