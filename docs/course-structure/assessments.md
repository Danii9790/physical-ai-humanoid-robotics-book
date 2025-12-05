---
sidebar_position: 2
---

# Assessments

## Overview

This course employs a project-based assessment approach designed to reinforce theoretical concepts with practical application. Each assessment builds upon the skills and knowledge acquired in previous modules.

## Assessment Breakdown

| Assessment | Module | Weight | Due Week |
|------------|--------|--------|----------|
| ROS 2 Package Project | Module 1 | 20% | Week 4 |
| Digital Twin Simulation | Module 2 | 25% | Week 7 |
| Isaac Perception Pipeline | Module 3 | 25% | Week 12 |
| Capstone Humanoid Project | Module 4 | 30% | Week 13 |

---

## 1. ROS 2 Package Project (20%)

### Objective
Demonstrate proficiency in fundamental ROS 2 concepts by designing, implementing, and testing a modular ROS 2 package.

### Requirements

**Minimum Components:**
- At least 3 nodes with clear responsibilities
- Topic-based communication (minimum 2 topics)
- At least 1 service OR 1 action
- Proper package structure with dependencies

**Technical Requirements:**
- Python or C++ implementation
- Launch file to start all nodes
- Configuration files (YAML) for parameters
- Error handling and logging

**Documentation:**
- README with system overview
- Architecture diagram showing node communication
- Code comments explaining functionality
- Usage instructions

### Example Project Ideas

1. **Simulated Robot Controller**
   - Keyboard input node
   - Velocity command publisher
   - Simulated robot state node
   - Visualization node

2. **Sensor Data Pipeline**
   - Sensor data generator
   - Data processing node
   - Data visualization node
   - Data logging service

3. **Multi-Agent Coordinator**
   - Multiple robot simulation nodes
   - Coordination logic node
   - Collision avoidance service
   - Status monitoring node

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| Architecture | 30% | Clear node design, proper separation of concerns |
| Communication | 30% | Correct use of topics, services, or actions |
| Code Quality | 20% | Clean code, documentation, error handling |
| Functionality | 20% | System works as intended, meets requirements |

### Submission

- GitHub repository link
- README with setup instructions
- 2-3 minute demonstration video
- Brief report (2-3 pages) explaining design decisions

---

## 2. Digital Twin Simulation (25%)

### Objective
Develop and interact with a functional digital twin of a humanoid robot in a simulated environment.

### Requirements

**Robot Model:**
- URDF/XACRO description with minimum 6 DOF
- Accurate mass and inertia properties
- Visual and collision geometries
- At least 2 sensor types (e.g., camera + IMU)

**Simulation Environment:**
- Gazebo OR Unity world
- Minimum 3 environmental obstacles
- Textured ground plane
- Proper lighting

**Control Implementation:**
- Joint position or velocity control
- Keyboard or GUI control interface
- Stable operation for at least 60 seconds
- Emergency stop functionality

**Sensor Integration:**
- ROS 2 topics publishing sensor data
- Sensor visualization (RViz or custom)
- Data logging capability

### Deliverables

1. **Code Package**
   - Complete ROS 2 package
   - URDF/XACRO files
   - World/scene files
   - Launch files

2. **Documentation**
   - System architecture diagram
   - Robot specifications (DOF, sensors, mass)
   - Setup and launch instructions
   - Known limitations

3. **Demonstration**
   - 3-5 minute video showing:
     - Robot model in simulation
     - Control interface
     - Sensor data visualization
     - Basic movements

### Evaluation Criteria

| Criterion | Points | Description |
|-----------|--------|-------------|
| Model Quality | 25 | Accurate kinematics, proper URDF structure |
| Simulation Fidelity | 25 | Realistic physics, sensor behavior |
| Control Implementation | 25 | Responsive, stable control |
| Documentation | 15 | Clear, complete, professional |
| Demonstration | 10 | Effective showcase of capabilities |

---

## 3. Isaac Perception Pipeline (25%)

### Objective
Implement and evaluate a perception pipeline using NVIDIA Isaac Sim for a humanoid robot.

### Requirements

**Part 1: VSLAM Implementation (40%)**
- Isaac Sim scene with humanoid robot
- Camera sensor configuration (RGB-D or stereo)
- VSLAM algorithm (ORB-SLAM3, RTAB-Map, or equivalent)
- Mapping of unknown environment
- Real-time localization

**Part 2: Nav2 Integration (40%)**
- Nav2 configuration for bipedal locomotion
- Costmap setup with appropriate parameters
- Global and local planners configured
- Goal-oriented navigation demonstration
- Dynamic obstacle handling

**Part 3: Performance Analysis (20%)**
- Map quality metrics (coverage, accuracy)
- Localization error analysis
- Navigation success rate
- Computational performance metrics

### Deliverables

1. **Implementation**
   - Isaac Sim scene files
   - ROS 2 packages for VSLAM and Nav2
   - Configuration files
   - Launch files

2. **Analysis Report** (5-7 pages)
   - Methodology description
   - Experimental setup
   - Results with graphs and tables
   - Discussion of findings
   - Challenges and solutions

3. **Demonstration Video** (5-7 minutes)
   - System overview
   - VSLAM mapping process
   - Navigation to multiple goals
   - Obstacle avoidance
   - Performance metrics visualization

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| VSLAM Accuracy | 25% | Map quality, localization precision |
| Nav2 Configuration | 25% | Appropriate parameters for humanoid |
| Navigation Performance | 25% | Success rate, path quality, obstacle handling |
| Analysis & Documentation | 25% | Thorough evaluation, clear reporting |

---

## 4. Capstone Humanoid Project (30%)

### Objective
Integrate knowledge and skills from all modules to develop an intelligent humanoid robot capable of performing a complex, multi-step task in a simulated environment.

### Requirements

**System Integration:**
- ROS 2 communication framework
- Isaac Sim simulation environment
- VSLAM for localization
- Nav2 for navigation
- VLA pipeline (Vision-Language-Action)

**Core Capabilities:**

1. **Locomotion (25%)**
   - Stable bipedal walking
   - Navigate to specified locations
   - Avoid dynamic obstacles
   - Recovery from disturbances

2. **Perception (20%)**
   - Visual SLAM
   - Object detection and recognition
   - Scene understanding
   - Spatial reasoning

3. **Voice Interface (20%)**
   - Speech recognition (Whisper)
   - Natural language command parsing
   - Spoken feedback (text-to-speech)
   - Dialogue management

4. **Task Planning (20%)**
   - GPT-based task decomposition
   - Multi-step plan execution
   - Error handling and recovery
   - Adaptive replanning

5. **Manipulation (15%)**
   - Pick and place objects
   - Grasp planning
   - Collision avoidance
   - Force control (if applicable)

### Task Scenarios (Choose One)

1. **"Clean up the living room"**
   - Detect misplaced objects
   - Plan pickup sequence
   - Navigate and grasp objects
   - Place in designated locations

2. **"Bring me a water bottle from the kitchen"**
   - Navigate to kitchen
   - Locate and identify water bottle
   - Grasp and carry safely
   - Return and hand over to person

3. **"Set the table for dinner"**
   - Understand table setting requirements
   - Locate plates, utensils, glasses
   - Arrange in proper positions
   - Handle multiple objects

4. **Custom Task** (requires approval)
   - Propose your own complex task
   - Must integrate all required capabilities
   - Submit proposal by Week 12

### Deliverables

1. **System Implementation**
   - Complete ROS 2 workspace
   - Isaac Sim scene and robot model
   - VLA pipeline code
   - Configuration files

2. **Documentation** (10-15 pages)
   - Executive summary
   - System architecture
   - Component descriptions
   - Integration approach
   - Challenges and solutions
   - Future improvements

3. **Demonstration Video** (5-7 minutes)
   - System overview
   - Task execution (full run)
   - Key features highlighted
   - Challenges encountered
   - Results and discussion

4. **Final Presentation** (15 minutes + 5 min Q&A)
   - Problem statement
   - Technical approach
   - System architecture
   - Live demonstration OR video
   - Results and evaluation
   - Lessons learned

### Evaluation Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| System Integration | 25% | All modules work together seamlessly |
| Task Completion | 25% | Successfully completes assigned task |
| Robustness | 20% | Handles errors, edge cases, disturbances |
| Innovation | 15% | Creative solutions, extensions beyond requirements |
| Documentation & Presentation | 15% | Clear, comprehensive, professional |

---

## Submission Guidelines

### General Requirements

- **Code:** Well-commented, follows style guidelines
- **Documentation:** PDF format, proper citations (APA 7th edition)
- **Videos:** MP4 format, 1080p minimum, clear audio
- **Repository:** Public GitHub repository with README

### Late Submission Policy

- **Within 24 hours:** 10% penalty
- **24-48 hours:** 25% penalty
- **Beyond 48 hours:** Not accepted (except with prior approval)

### Academic Integrity

- All work must be your own
- Properly cite all sources and references
- Collaboration is encouraged for learning, but submissions must be individual
- Use of AI tools (ChatGPT, Copilot) must be disclosed

---

## Grading Scale

| Grade | Percentage | Description |
|-------|------------|-------------|
| A | 90-100% | Exceptional work, exceeds expectations |
| B | 80-89% | Good work, meets all requirements |
| C | 70-79% | Satisfactory work, meets most requirements |
| D | 60-69% | Minimal work, meets some requirements |
| F | &lt;60% | Unsatisfactory work |


---

**Navigation:**  
← [Weekly Schedule](./weekly-schedule.md) | [Hardware Architecture →](../hardware-architecture/overview.md)
