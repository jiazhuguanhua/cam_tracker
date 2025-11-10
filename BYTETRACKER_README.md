# ByteTracker Integration for Cam Tracker

## 🎯 Overview

The cam_tracker node now integrates **ByteTracker** for robust object tracking with persistent IDs.

## ✨ Key Features

### 1. **Ball Tracking (Mode 1 - Pickup Zone)**
- 🎾 **Persistent ID Tracking**: Each ball gets a unique tracking ID
- 🔄 **Smart ID Switching**: When the tracked ball is lost, automatically selects the closest ball based on position
- 📍 **Position Memory**: Remembers last known position to maintain tracking continuity
- ⏱️ **Lost Frame Tolerance**: Allows up to 10 frames of ID loss before switching targets

### 2. **Car Tracking (Mode 2 - Release Zone)**
- 🚗 **Multi-object Tracking**: Tracks multiple cars simultaneously with unique IDs
- �� **ID-sorted Output**: Cars are sorted by tracking ID for consistent ordering

### 3. **Tracking Strategy**

```
Priority 1: Continue tracking same ID
    ↓ (ID lost)
Priority 2: Select closest ball by position (distance < 100 pixels)
    ↓ (Too far or lost too long)
Priority 3: Select ball with highest confidence
```

## 📋 Message Format

### CamTrack.msg (Updated)
```msg
# Ball tracking (Mode 1)
int32 ball_num              # Number of balls detected
int32 ball_id               # Tracking ID of selected ball (-1 if not tracking)
float32 ball_x              # X coordinate of tracked ball
float32 ball_y              # Y coordinate of tracked ball

# Car tracking (Mode 2)
int32 car_num               # Number of cars detected
int32[] car_ids             # Tracking IDs of all cars
float32[] car_x             # X coordinates of all cars
float32[] car_y             # Y coordinates of all cars
```

## 🔧 Configuration Parameters

In your launch file or via rosparam:

```xml
<node name="cam_tracker_node" pkg="cam_tracker" type="cam_tracker_node.py">
    <!-- Tracker type: 'bytetrack' or 'botsort' -->
    <param name="tracker_type" value="bytetrack" />
    
    <!-- Confidence threshold for detection -->
    <param name="confidence_threshold" value="0.5" />
</node>
```

## 🎮 Control Modes

```bash
# Stop tracking
rostopic pub /cam_tracker/tracker_control std_msgs/UInt8 "data: 0"

# Start ball tracking (pickup mode)
rostopic pub /cam_tracker/tracker_control std_msgs/UInt8 "data: 1"

# Start car tracking (release mode)
rostopic pub /cam_tracker/tracker_control std_msgs/UInt8 "data: 2"
```

## 📊 Monitoring

### View tracking info:
```bash
rostopic echo /cam_tracker/info
```

### Check logs:
```bash
# See tracking status
rosnode log /cam_tracker_node

# Example log outputs:
# [Ball Tracking] Start tracking ID=3
# [Ball Tracking] Continue tracking ID=3
# [Ball Tracking] ID switch: 3 -> 5 (distance=45.2)
# [Ball Tracking] Lost too long, new target ID=7
```

## 🖼️ Visualization

The visualizer now displays tracking IDs:

- **Status Bar**: Shows ball count and current tracking ID `Ball: 1 [ID:3]`
- **Ball Label**: `Ball ID:3 (312, 401)`
- **Car Labels**: `Car ID:5 (450, 300)`

## 🔍 Tracking Behavior

### Example Scenario:
1. **Frame 1-50**: Tracking ball with ID=3 at position (300, 400)
2. **Frame 51**: Ball ID=3 temporarily lost
3. **Frame 52**: Found ball ID=5 at position (310, 405) - only 14 pixels away
4. **System Decision**: Switch to ID=5 (closest match)
5. **Output**: `[Ball Tracking] ID switch: 3 -> 5 (distance=14.1)`

### Lost Frame Handling:
- Tolerance: Up to **10 frames** (adjustable via `self.max_lost_frames`)
- If lost > 10 frames: Select highest confidence ball as new target
- Distance threshold: **100 pixels** (adjustable in code)

## 🚀 Advantages

✅ **Consistent Target**: Always tracks the same ball (unless intentionally switched)
✅ **Occlusion Handling**: Maintains tracking through brief occlusions
✅ **Smooth Transitions**: Intelligent ID switching based on spatial proximity
✅ **Multi-object Support**: Car tracking handles multiple objects seamlessly
✅ **Real-time Performance**: ByteTracker is optimized for speed

## 📝 Notes

- ByteTracker uses **Kalman filtering** for motion prediction
- Tracking IDs persist across frames using **IoU + motion matching**
- The tracker automatically handles ID assignment and lifecycle
- Mode switching automatically resets tracking state

## 🐛 Debugging

Enable debug logging:
```bash
rosservice call /cam_tracker_node/set_logger_level ros.cam_tracker_node DEBUG
```

This will show detailed tracking decisions in the console.

---
**Author**: Cam Tracker Team  
**Date**: 2025-11-10  
**Version**: 2.0 with ByteTracker
