# Ball-on-Plate Control System using Observers and Kalman Filtering

## Overview

This project focuses on modeling, simulation, state estimation, and control of a ball-on-plate system. The system is nonlinear and unstable, making it a suitable platform for studying feedback control, observer design, Kalman filtering, and embedded implementation.

The main objective was to estimate the ball position and velocity using camera-based measurements and improve system behavior using model-based observers and Kalman filtering techniques.

## Project Motivation

The ball-on-plate system is a classical control engineering problem that demonstrates the challenges of controlling nonlinear, unstable, and coupled dynamic systems. Since only the ball position is directly measured, velocity and internal system states must be estimated using observers or filtering techniques.

This project helped me understand the complete workflow from system modeling to observer design, Kalman filtering, simulation, and implementation-oriented C code.

## Key Features

- Mathematical modeling of the ball-on-plate system
- Open-loop simulation of ball dynamics
- 2nd-order Luenberger observer design
- 3rd-order observer including actuator dynamics
- Linear Kalman Filter implementation
- Extended Kalman Filter implementation for nonlinear estimation
- MATLAB simulation and result visualization
- C implementation for embedded-system-oriented deployment
- Comparison between observer-based and Kalman-filter-based estimation

## System Description

The ball-on-plate system consists of a ball moving on a tilting plate. The plate angle is controlled using servo motors. A camera system measures the ball position, while the velocity and additional states are estimated using observer-based methods.

The system includes:

- Ball dynamics
- Plate angle dynamics
- Servo actuator behavior
- Camera-based ball position measurement
- State estimation using observers and Kalman filters

## Methods Implemented

### 1. Open-Loop Simulation

The first step was to simulate the ball motion without feedback control. This helped in understanding the natural unstable behavior of the system.

### 2. Luenberger Observer

A Luenberger observer was implemented to estimate unmeasured states such as ball velocity from measured ball position.

### 3. 3rd-Order Observer

The observer was extended by including actuator/servo dynamics, allowing a more realistic representation of the physical system.

### 4. Linear Kalman Filter

A Linear Kalman Filter was implemented to estimate the system states while considering process noise and measurement noise.

### 5. Extended Kalman Filter

An Extended Kalman Filter was implemented for the nonlinear system model. The nonlinear equations were linearized using the Jacobian matrix during each prediction step.

## Tools and Technologies

- MATLAB
- C programming
- Control systems
- State-space modeling
- Luenberger observer
- Kalman Filter
- Extended Kalman Filter
- Embedded systems preparation

## Repository Structure

```text
Matlab_Simulation/       MATLAB simulations and analysis scripts
c_code/       C implementation of observer and Kalman Filter logic
results/      Plots, figures, and performance comparison
media/        System images, GIFs, and demo visuals
