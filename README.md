# AquaTriV Dataset & DevKit

**AquaTriV** is a large-scale, multi-scene, multi-modal underwater dataset designed for **dense SLAM**, **multi-sensor fusion**, and **neural reconstruction**.

It integrates **active, passive, and neuromorphic vision**, together with **inertial, acoustic, and pressure sensing**, and provides both:

- ✅ High-precision **dense mapping ground truth**
- ✅ Continuous **6-DoF localization ground truth**

👉 Project Page: https://water-scanner.github.io/AquaTriV.github.io/

---

## 🌊 1. Overview

Underwater environments pose significant challenges due to light attenuation, turbidity, and scattering effects. Existing datasets often suffer from:

- ❌ Lack of dense mapping ground truth  
- ❌ Incomplete or discontinuous trajectories  
- ❌ Limited sensing modalities  
- ❌ Restricted to single scenarios  

**AquaTriV addresses these limitations by:**

- 🌐 Multi-scene: Pool, Cave, River, Sea  
- 👁️ Triple-vision:  
  - Passive (Monocular & Stereo)  
  - Active (3D Laser Scanner)  
  - Neuromorphic (Event Camera)  
- 📡 Multi-sensor fusion: IMU, DVL, Pressure  
- 📍 Accurate localization GT:  
  - Indoor: Motion Capture System (MCS)  
  - Outdoor: INS + Colmap (OpenGT)  
- 🧱 Dense mapping GT:  
  - Millimeter-level 3D laser reconstruction  

---

## 📊 2. Comparison with Existing Underwater Datasets

| Dataset | Scenario | Vision | Other Sensors | Open | Localization GT | Mapping GT |
|--------|----------|--------|---------------|------|-----------------|------------|
| [Abandoned Marina](http://eia.udg.es/~dribas/) | Harbor | ✗ | IMU, Acoustic, Pressure | ✓ | GPS (Surface) | ✗ |
| ARACATI | River | Monocular | IMU, Acoustic | ✗ | DGPS | ✗ |
| [Caves Sonar](http://cirs.udg.edu/caves-dataset/) | Cave | Monocular | IMU, Acoustic, Pressure | ✓ | Landmarks | ✗ |
| [AQUALOC](http://www.lirmm.fr/aqualoc/) | Sea | Monocular | IMU, Pressure | ✓ | Colmap | ✗ |
| Inspection | Tank | Stereo | IMU, Pressure | ✗ | MCS | ✗ |
| [HAUD](https://bat.sjtu.edu.cn/zh/haud-dataset/) | Pool | Stereo | IMU | ✓ | MCS | ✗ |
| [AURORA](https://ieee-dataport.org/open-access/aurora-multi-sensor-dataset-robotic-ocean-exploration) | Sea | Monocular | IMU, Acoustic, Pressure | ✓ | INS | ✗ |
| Seaward | River | ✗ | IMU, Acoustic, Pressure | ✗ | GPS | ✗ |
| [Eiffel Tower](https://www.seanoe.org/data/00810/92226/) | Sea | Monocular | IMU | ✓ | Colmap | Sparse |
| [Tank](https://senseroboticslab.github.io/underwater-tank-dataset/) | Tank | Stereo | IMU, Acoustic, Pressure | ✓ | AprilTag | ✗ |
| **[AquaTriV (Ours)](https://water-scanner.github.io/AquaTriV.github.io/)** | Pool, Cave, River, Sea | Mono + Stereo + Event + Laser | IMU, Acoustic, Pressure | ✓ | MCS + INS+Colmap | **Dense** |

---

## 🚀 3. Key Features

- 🧠 **First underwater dataset with triple-vision fusion**  
- 📍 **Continuous 6-DoF ground truth trajectories**  
- 🧱 **High-fidelity dense point cloud ground truth**  
- 🌊 **Cross-domain real-world scenarios**  
- ⚡ **Supports event-based perception research**  
- 🎯 **Benchmark for dense SLAM & neural rendering (3DGS)**  

---

## 📦 4. Dataset Statistics

- 📁 25 sequences  
- 💾 ~250 GB data  
- ⏱️ ~7 hours duration  
- 📏 ~2.8 km trajectories  
- 🌗 Multiple illumination conditions  
- 🌫️ Varying turbidity & flow  

---

## 🛠️ 5. Development Kit

We provide a complete DevKit for:

### 🔹 Point Cloud Generation
- Laser-based 3D reconstruction  
- Refraction-aware modeling  
- Motion distortion correction (ESKF)

### 🔹 Localization Evaluation
- ATE60 / RTE  
- CRS (Coverage Ratio Score)  
- TCR (Trajectory Continuity Ratio)

### 🔹 Mapping Evaluation
- Accuracy / Completeness  
- Chamfer Distance  
- Map Entropy  

---

## 🧪 6. Supported Research Tasks

- Underwater SLAM  
- Multi-sensor fusion  
- Dense reconstruction  
- Event-based vision  
- Neural rendering (3D Gaussian Splatting)  
- Underwater perception  

---

## 📂 7. Data Format

- ROS bag format  
- Multi-modal synchronized topics  
- Raw + processed data available  

---

## 📥 8. Download

👉 Coming soon...

---

## 📜 9. Citation

If you use this dataset, please cite:

```bibtex
@article{aquaTriv2026,
  title={AquaTriV: An Underwater Multi-Scene Multi-Modal Dense SLAM Dataset},
  author={Ou, Yaming et al.},
  journal={To Be Assigned},
  year={2026}
}
