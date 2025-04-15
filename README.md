# Fast Human-Robot Handover using a Vision-based pipeline 

The system was developed for the 2024 Robotic Grasping and Manipulation Competition ([RGMC - Sub-Track \#4 on Human-to-Robot Handovers](https://corsmal.eecs.qmul.ac.uk/rgm24icra/)), and the code is herein released for reproducibility. 

This repository provides the packages to perform vision-based human to robot fast handover. The code is tested on **Ubuntu 20.04** with **ROS Noetic**. 

Our setup utilizes two computers connected to the same local network, one for the **vision pipeline**, and the other for **robot control**. For the setup of the PC running the vision pipeline, please refer to the relative [guide](https://github.com/SIRSIIT/fast-handover/blob/main/handover_vision/README.md), while for the robot control, please refer to [this one](https://github.com/SIRSIIT/fast-handover/blob/main/handover_robot/README.md). 

## Network Configuration

This system assumes the following network layout:

- The two PCs are connected to the `172.31.1.*` subnet.
- The robot is on a separate network: `172.32.1.*`.

> ⚙️ You can optionally adjust these settings if needed.


### Vision PC (ROS Master):
- **IP Address:** `172.31.1.1`  
- **Netmask:** `255.255.255.0`

### Robot Control PC:
- **IP Address:** `172.31.1.2`  
- **Netmask:** `255.255.255.0`

In this setup, the **vision PC** serves as the **ROS master**, running `roscore`.

---

### ROS Environment Setup

If you change any IP addresses, make sure to update your `~/.bashrc` file on both machines and inside Docker containers:

```bash
export ROS_MASTER_URI=http://172.31.1.1:11311/
export ROS_IP=172.31.1.1
```

Then, apply the changes:

```bash
source ~/.bashrc
```
---

# Video of the RGMC ICRA 24 competition

[![RGMC ICRA 24 - Handover](https://img.youtube.com/vi/4wzR3eVm9P0/0.jpg)](https://www.youtube.com/watch?v=4wzR3eVm9P0)
