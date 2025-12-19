# Rescue People (Unibotics) — Autonomous Spiral Search

This repository contains the solution to a **Unibotics rescue problem** in which an autonomous UAV searches for people using a **center-anchored spiral trajectory** and a **vision-based detection system**. Detected people (victims) are identified through face detection on the ventral camera feed, and their positions are registered during the mission.

**Main script:** `Erik_Artigas_Reverter_Rescue_People.py`

---

## Repository Contents

- `Erik_Artigas_Reverter_Rescue_People.py`
- `README.md`

---

## Execution Environment (Unibotics)

The solution is designed to run inside **Unibotics**, which provides the following APIs.

### Provided APIs

**HAL**
- `get_position()`
- `get_ventral_image()`
- `takeoff()`
- `land()`
- `set_cmd_vel(vx, vy, vz, yaw_rate)`
- `set_cmd_pos(x, y, z, yaw)`

**WebGUI**
- `showImage(image)`

### Required Python Libraries

- Python 3.x  
- `opencv-python`  
- `numpy`  

---

## Problem Description

The objective is to autonomously locate a predefined number of people in the environment. The UAV must explore the area efficiently, detect victims using onboard perception, register their positions, and safely return to the starting point once the mission is completed.

No user interaction is required once execution starts.

---

## Mission Logic

The mission is implemented as a **state-based control loop** with the following phases:

1. **TAKEOFF**  
   The UAV takes off and climbs until reaching a predefined cruise altitude.

2. **TRAVEL TO SEARCH CENTER**  
   The UAV navigates to a predefined search center `(CENTER_X, CENTER_Y)` while maintaining altitude.

3. **SEARCH (Spiral Exploration)**  
   - The UAV follows an **Archimedean spiral** centered at the search point.  
   - The spiral progressively expands to cover an increasing area.  
   - The ventral camera feed is continuously analyzed to detect faces.

4. **VICTIM REGISTRATION**  
   - When a face is detected, the UAV position `(x, y)` is recorded.  
   - The position is compared with previous detections to avoid duplicates.

5. **MISSION COMPLETION**  
   - Once the maximum number of victims is reached, a summary is printed.  
   - The UAV returns to `(0, 0)` and lands.

---

## Spiral Search Strategy

The spiral trajectory is an **Archimedean spiral**:

r(θ) = r₀ + k · θ

Where:
- `r₀` is the initial radius,  
- `k` controls spiral growth,  
- `θ` is the accumulated angular displacement.

Angular velocity is adjusted dynamically:

ω = v / r

This ensures a smooth and controlled outward sweep.

---

## Victim Detection

Victim detection uses **OpenCV Haar cascade face detection**.  
To improve robustness against orientation changes, the grayscale image is evaluated at multiple rotation angles. A detection in any rotated version is considered valid.

---

## Output

During execution, the script prints mission events, including:
- Center reached / spiral start
- Each newly detected victim position
- Final mission report listing all registered victims

---

## Notes / Assumptions

- Victim positions are approximated using the UAV position at detection time.  
- No manual input is required once the mission starts.  
- The solution relies exclusively on Unibotics-provided APIs.  
