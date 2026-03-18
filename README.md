# Moveover: V2N-Based Algorithm and Communication Protocol for Autonomous Non-Stop Intersections

This repository contains the official implementation for the paper: **"V2N-Based Algorithm and Communication Protocol for Autonomous Non-Stop Intersections"**.


## Overview

**Moveover** (Seamless Mobility of Vehicles Over Intersections) is a novel algorithm and Vehicle-to-Network (V2N) communication protocol designed to allow Connected and Autonomous Vehicles (CAVs) to cross intersections without stopping. 

Unlike computationally heavy centralized controllers, Moveover delegates trajectory and speed profile selection to individual vehicles, allowing each CAV to optimize its trajectory based on its unique kinematic characteristics. A local intersection controller runs in tandem to prevent collisions through deterministic conflict zone reservations.

## Repository Structure

| Directory / File | Description |
| :--- | :--- |
| `four_way_single_lane/` | Source code for 4-way single-lane intersection simulations. |
| `four_way_two_lane/` | Source code for 4-way multi-lane intersection simulations. |
| `three_way_single_lane/` | Source code for T-junction (3-way) simulations. |
| `roundabout/` | Source code for roundabout simulations. |
| `sumocfg/` | SUMO network (`.net.xml`), route (`.rou.xml`), and configuration files. |
| `results/` | Output directory containing generated data (emissions, travel times, etc.). |
| `helper.py` | Shared utility and helper functions for the main simulation scripts. |
| `README.md` | Project documentation. |

## Prerequisites

To run these simulations, ensure your environment meets the following requirements:

* **Python:** 3.14 (Tested)
* **SUMO:** 1.21.0
* **Python Packages:**
    * `traci` (SUMO's Traffic Control Interface)
    * `bisect` (Python standard library)

## Simulation Kinematics & Rules

The Moveover algorithm relies on specific physical and behavioral constraints for the simulated vehicles:

* **Vehicle Length:** 5 meters
* **Minimum Safety Distance:** 2 meters (breaching this registers as a collision)
* **Minimum Crossing Speed:** 6 m/s through the intersection
* **Acceleration rate:** 2.6 m/s²
* **Deceleration rate:** 4.5 m/s²
* **EGO Vehicle:** Defined as the current vehicle being scheduled; "ahead" vehicles refer to those spatially in front of the EGO vehicle.
* **SUMO Speed Modes:** Custom speed modes (e.g., 54, 31) are enforced via TraCI to override default SUMO behaviors when Moveover is active. See [https://sumo.dlr.de/docs/index.html](https://sumo.dlr.de/docs/index.html) for further details.

### Backup Mode (Fallback Mechanism)
If a vehicle's scheduled trajectory requires its speed to drop below **3 m/s** at any point, the system triggers **Backup Mode**. This turns off the Moveover algorithm for that specific vehicle, handing control back to SUMO's default car-following and intersection-handling models to ensure safety.

## How to Run

Simulations are executed by running the `main.py` module for your desired intersection layout. 

**Example:** Running the four-way single-lane simulation with the SUMO GUI enabled.

```bash
python -m four_way_single_lane.main --sumo_gui "C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
```

## If you find this work useful in your research, please consider citing:
```
@misc{farina2026v2nbasedalgorithmcommunicationprotocol,
      title={V2N-Based Algorithm and Communication Protocol for Autonomous Non-Stop Intersections}, 
      author={Lorenzo Farina and Lorenzo Mario Amorosa and Marco Rapelli and Barbara Mav\'i Masini and Claudio Casetti and Alessandro Bazzi},
      year={2026},
      eprint={2603.05165},
      archivePrefix={arXiv},
      primaryClass={cs.NI},
      url={https://arxiv.org/abs/2603.05165}, 
}
```