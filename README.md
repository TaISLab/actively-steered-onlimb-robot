# Actively Steered Open On-Limb Robot 

On-limb robot designed to move along compliant cylindrical surfaces, such as human limbs or pipes. The platform integrates active steering, proprioceptive sensing, embedded control electronics, and compliant mechanical adaptation for locomotion on variable-diameter surfaces.

This repository accompanies the paper:

> Tobar-Subía-Contento, L.M.; Valenzuela, J.; Mandow, A.; Gómez-de-Gabriel, J.M. **"Actively Steered Open On-Limb Robot with Alignment Control and Online Diameter Estimation."** *Electronics* **2026**, *15*(17), 4028. https://doi.org/10.3390/electronics15174028

The main contribution is an integrated open on-limb robotic platform with active steering for real-time locomotion and alignment control. A generalized kinematic framework based on the complete Jacobian of the roller centers provides a geometric alignment variable for closed-loop steering correction and enables coordinate-independent online estimation of the supporting surface diameter from proprioceptive measurements.

Experiments on variable-diameter surfaces demonstrate the robot's ability to maintain centered longitudinal locomotion, handle non-symmetric link deflections, and estimate the local diameter online.

## Repository Contents

The repository is organized into two main sections:

```text
repository/
├── CAD/
├── electronics/
├── ControlSoftware/
├── MICIU+Cofinanciado+AEI.jpg
├── README.md
├── CITATION.cff
└── LICENSE
```

## Mechanical Design

The `mechanical/STL/` directory contains the mechanical components required to fabricate the robot structure using **additive manufacturing (3D printing)**.

The structural components of the prototype reported in the associated publication were fabricated using **polylactic acid (PLA)**.

### 3D Printing

Printing parameters may need to be adjusted depending on the printer, material, nozzle, layer height, and desired mechanical properties.

Users should verify dimensional tolerances and mechanical compatibility before final assembly.

## Electronics

The `electronics/` directory contains the electronic design files associated with the wearable robot prototype.

The electronics were designed using **KiCad** and the directory includes the complete project required to inspect the electrical schematic and PCB layout:

```text
electronics/
├── proyecto_robot_limbot.kicad_pro
├── proyecto_robot_limbot.kicad_sch
└── proyecto_robot_limbot.kicad_pcb
```

The files correspond to:

* **`.kicad_pro`** — KiCad project configuration.
* **`.kicad_sch`** — electronic schematic of the system.
* **`.kicad_pcb`** — printed circuit board (PCB) layout.

The project can be opened directly in **KiCad** using the `.kicad_pro` file.

Temporary files, local configuration files, autosaves, and backup directories generated automatically by KiCad are not included in the repository.

The electronic design files are provided to support reproduction, inspection, and further development of the hardware used in the research prototype.

# Control Software

This folder contains the software used for the control and experimental operation of the actively-steered on-limb robot.

The embedded control code was implemented for the ESP32-based controller and includes motor position control, servo control, sensor acquisition, signal filtering, serial communication, and experimental data transmission.

```text
ControlSoftware/
├── actively-steered-onlimb-robot.ino
├── robot_control_interface.py
```

## Citation

If you use or adapt the files, mechanical designs, or electronic designs contained in this repository for academic or research purposes, please cite the associated publication (see reference above) and, when appropriate, this repository.

GitHub users can use the **“Cite this repository”** option, or refer directly to the `CITATION.cff` file included in this repository, which contains the complete bibliographic metadata for both the article and the repository.

## Funding

<img src="MICIU+Cofinanciado+AEI.jpg" alt="MICIU/AEI logo" width="300">

This work is part of project PID2021-127221OB-I00 (CONCERTO — Control Colaborativo para Interacción física Empática entre RoboT y humanO), funded by MICIU/AEI/10.13039/501100011033/FEDER, UE.

## Acknowledgment

This repository accompanies academic research on wearable robotic systems and is provided to facilitate reproducibility and further research in the field.
