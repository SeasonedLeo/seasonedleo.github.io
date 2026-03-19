---
layout: page
title: Predictor-Compensated CLF-QP for Vehicle Tracking with Sensing and Actuation Delays
description: Delay-aware control for teleoperated vehicle tracking using CLF-QP with SQP.
img: assets/img/12.jpg
importance: 1
category: work
related_publications: true
---

## Overview

This project studies delay-aware vehicle tracking for teleoperation. The controller combines predictor compensation with a control Lyapunov function quadratic program (CLF-QP) solved via SQP to enforce stability and constraints under sensing and actuation delays.

## Agenda

- Background and motivation
- Problem statement
- Literature review
- Algorithm overview
- Results and discussion
- Summary and future work

## Background and Motivation

Modern control systems experience delays from sensing pipelines, communication networks, computation, and actuator dynamics. In safety-critical settings like autonomous driving, teleoperation, V2X communication, and multi-vehicle coordination, these delays can degrade tracking and compromise stability.

Existing methods have gaps when **measurement delays** and **actuation delays** occur simultaneously.

- Predictor feedback ensures stability but does not directly enforce stability constraints.
- Delay-aware barrier approaches emphasize invariance but lack explicit performance guarantees and often treat only a single delay channel.

## Problem Statement

**Objective**: Compute an optimal actuation command using CLF-SQP under state and input delays.

### System and Optimization

- Delay-aware system model with sensing and actuation delays
- Optimization problem with box and rate constraints

## Optimization Solver: SQP

### Decision Variables

- Control inputs and slack variables

### Constraints and Bounds

- Amplitude and rate limits
- Rate limits referenced to previous command
- CLF inequality constraint
- No equality constraints

### Objective

- Quadratic cost with tracking and effort penalties

### Warm Start

- Initialize with the previous solution to reduce iterations

## Algorithm

- Predictor estimates the arrival state
- CLF-QP computes control consistent with Lyapunov decrease
- SQP solves a small QP each cycle

## Simulation Setup

- Teleoperation tracking scenario with known and unknown delays

## Results and Discussion

### True Delay Known

- Stable tracking with CLF-QP
- Constraint compliance under delay

### True Delay Unknown

- CLF-QP remains resilient to delay estimation bias
- Predictive control preserves stability trends

## Conclusion

- **Objective and guarantees**: CLF-QP enforces Lyapunov decrease directly, providing an explicit stability certificate under delay.
- **Constraint handling**: CLF-QP respects amplitude and rate limits through box constraints; PID saturates after the fact.
- **Delay handling**: Both approaches use a predictor, but CLF-QP solves for commands consistent with the predicted arrival state and Lyapunov decrease.
- **Computation**: The SQP-based QP is small and converges reliably at high update rates (e.g., 200 Hz).

## Summary of Learning

- Lyapunov-centric objective yields formal stability decay; slack controls strictness.
- With PSD/PD weights and affine constraints, the QP remains convex and small.
- Slack preserves feasibility and tunes the trade-off between stability and constraint satisfaction.

## Future Work

- Joint CLF-CBF optimization to enforce performance and safety simultaneously
- Robustness to model uncertainty and disturbances
- Extend from kinematic bicycle to nonlinear dynamics with adapted predictor and constraints

## Acknowledgments

Thank you. Questions welcome.
