# UAV Strategic Deconfliction System  
### FlytBase Robotics Assignment – 2025  

This project implements a **Strategic UAV Deconfliction Authority** that verifies whether a drone’s planned waypoint mission is safe to execute in shared airspace.  
The system performs **spatial + temporal conflict detection**, explains conflicts, and visualizes flight paths in **4D (3D space + time)**.

---

## 🚀 Features
✔️ Supports **Primary Drone Mission Validation**  
✔️ Multiple **Simulated Scheduled Flights**  
✔️ **Spatial Safety Check** (minimum separation buffer)  
✔️ **Temporal Alignment Logic** (time window overlap handling)  
✔️ **Conflict Explanation**
- Which drone?
- When?
- Where in space?
- Distance separation  
✔️ **4D Visualization using Plotly**  
✔️ **Unit-like sanity checks included**  
✔️ Clean modular code structure  

---

## 🧠 System Architecture

### 1️⃣ DroneTrajectory
Represents a UAV flight path with:
- 3D waypoints: `[x, y, z]`
- Constant-speed motion model
- Linear interpolation using SciPy
- Ability to query exact 3D position at any time `t`

### 2️⃣ DeconflictionService
Acts as a **central authority**:
- Maintains scheduled flight paths
- Validates a new primary mission
- Samples flight paths at fixed time resolution
- Aligns different mission start times
- Detects conflicts

Returns either:
```
("CLEAR", [])
```
or
```
("CONFLICT", [ ... conflict details ... ])
```

---

## ⚙️ How It Works

1️⃣ Assign travel time to waypoints using constant velocity  
2️⃣ Interpolate 3D positions over time  
3️⃣ Align global time between drones  
4️⃣ Sample positions every 0.5 seconds  
5️⃣ Compute Euclidean distance  
6️⃣ If separation < safety buffer → **conflict detected**  

Only the **first conflict per drone** is reported.

---

## 🛠️ Tech Stack
- Python
- NumPy
- SciPy
- Plotly
- Datetime

---

## 🧪 Sanity Tests
Before visualization runs, automatic checks verify:
- Expected conflict scenario detects correctly
- Safe scenario stays clear

Output:
```
Running sanity tests...
Sanity tests passed ✔️
```

---

## ▶️ Running The System

### 1️⃣ Install Dependencies
```
pip install numpy scipy plotly
```

---

### 2️⃣ Run Program
```
python main.py
```

---

## ✅ Example Console Output
```
--- Validating Mission: Primary_Alpha ---
Status: CONFLICT
ALERT: Potential collision with Drone_B_Cargo at 10.0s
Location: [50.0, 50.0, 20.0] | Separation: 0.5m
```

---

## 🎥 Visualization Output
An interactive HTML visualization is generated:
```
deconfliction_viz.html
```

Open it in any browser.

Shows:
- Primary mission path (Blue)
- Safe drone (Green)
- Conflicting drone (Red)
- Conflict markers (Orange)

---

## 🧩 Scalability Thoughts

### System Architecture Improvements
- Distributed microservices
- Real-time streaming ingestion (Kafka / MQTT)
- Spatial indexing (KD-Tree / R-Tree / Geohash)
- GPU acceleration
- Cloud-native + fault tolerance

### Algorithm Improvements
- Predictive motion modeling
- Adaptive sampling
- Multi-drone event resolution

---

## 🤖 AI Assistance Note
AI assisted with:
- Design guidance
- Debugging
- Documentation
- Code cleanup
- Visualization structuring

All logic was validated manually.

---

## 📌 Limitations
- Reports only first conflict per drone
- Constant-speed assumption
- Sampling-based rather than analytical

---

## 🏁 Status
✔️ Core System Ready  
✔️ Visualization Ready  
✔️ Testing Passed  
✔️ Submission Ready  

