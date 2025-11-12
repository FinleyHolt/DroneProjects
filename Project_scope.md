# Reasoning-Enabled Autonomy Framework for Small Unmanned Aerial Systems (sUAS)

**Submitted by:** 2ndLt Finley Holt
**Organization:** Delta Company, The Basic School, Quantico, VA
**Proposed Host Organization:** Marine Corps Tactical Systems Support Activity (MCTSSA), Digital Solutions Branch
**Date:** October 2025

## Background

Current small unmanned aerial systems (sUAS) follow predefined flight plans and require continuous operator supervision. They cannot interpret intent, reason about objectives, or adapt to changing conditions. These limitations constrain operational tempo and reduce effectiveness in communications-denied or bandwidth-restricted environments, where control links may be intermittent or unavailable.

Recent advances in embodied artificial intelligence have shown that Large Language Models (LLMs) can perform reasoning, task planning, and adaptive behavior in robotic systems. Projects such as Google DeepMind's RT-2 and PaLM-E, NVIDIA's Project GR00T, and OpenAI's Embodied GPT have demonstrated language-guided control and perception-grounded reasoning in manipulation and ground-robotic platforms. For aerial systems, however, comparable capability has only been demonstrated in simulation through Application Programming Interface (API) calls to cloud-hosted models—approaches that depend on persistent connectivity and are unsuitable for tactical, communications-denied environments.

The emergence of compact, power-efficient edge-compute devices such as the NVIDIA Jetson Orin and Jetson Thor, together with advances in lightweight large-language-model architectures, now makes it feasible to host a reasoning engine onboard a small tactical aircraft. These models enable onboard decision-making and planning with substantially lower computational and power demands, allowing real-time inference within the limits of embedded systems. This integration supports autonomous mission execution without external connectivity, maintaining effectiveness in degraded command-and-control conditions.

## Technical Approach

Under the Marine Corps Tactical Systems Support Activity (MCTSSA) Digital Solutions Branch, this project will develop a reasoning-enabled autonomy framework that converts natural-language mission intent into executable flight plans. A pretrained, locally hosted LLM-based reasoning engine will perform mission parsing, adaptive planning, and context-aware decision-making. The model will use a Mixture-of-Experts (MoE) architecture that maintains all expert weights on the device but activates only the relevant pathways at runtime, reducing compute and power demand while enabling responsive reasoning on embedded platforms.

Robot Operating System 2 (ROS 2) provides modular communication for autonomy software, and the MAVLink Software Development Kit (MAVSDK) offers a stable API for flight control and telemetry. With these systems handling low-level control, this project will focus on the reasoning, planning, and perception components that drive adaptive behavior.

## Operational Vignette

A reconnaissance team tasks a quadrotor: "Survey Named Area of Interest (NAI) 3 and identify vehicle movement before returning home." The onboard reasoning engine interprets this command into structured actions—navigating to the designated area, executing search patterns, detecting vehicles using onboard vision models, geotagging detections, and returning home. If communications are lost, the system continues the mission in accordance with the commander's intent. In response to environmental changes or obstacles, it replans locally to maintain mission continuity and aircraft safety under degraded conditions. This capability supports Force Design 2030 by enabling distributed, resilient sUAS operations that sustain autonomy in contested or communications-denied environments.

## Development and Testing

Development and evaluation will use PX4 Software-in-the-Loop (SITL) with the Gazebo simulation environment to assess mission performance, response latency, and robustness under dynamic conditions before live-flight testing. All frameworks—ROS 2, MAVSDK, PX4, and Gazebo—are open-source and freely available, ensuring transparent and reproducible development across Marine Corps platforms.

## Development Objectives

1. Implement natural-language parsing to translate mission intent into structured sub-tasks and update plans dynamically.

2. Develop a modular, containerized ROS 2-based software stack integrating reasoning, perception, and control through MAVSDK bridges.

3. Integrate onboard sensors for task verification, anomaly detection, and local replanning.

4. Validate reasoning reliability, latency, and mission robustness in PX4-SITL and Gazebo before live testing.

## Deliverables

1. Edge-deployable reasoning module and containerized software stack.

2. Simulation and live-flight demonstrations of autonomous mission execution from natural-language tasking.

3. Publishable research paper on the design and evaluation of a reasoning-enabled autonomy framework for sUAS, validated through simulation and live-flight testing.

## Collaboration and Impact

2ndLt Holt will work under the supervision of Shaun Monera with the MCTSSA Digital Solutions Branch, focusing on the design of the reasoning-enabled autonomy architecture, integration of the onboard AI system, and evaluation of autonomous system performance. This project will establish the Marine Corps' first reasoning-enabled drone architecture and advance embodied artificial intelligence for autonomous operations in contested, communications-denied environments.
