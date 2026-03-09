# 2D Pose Overlay Node

This node overlays 2D human pose keypoints and skeleton on the RGB camera image for visualization.

## Quick Start

```bash
# Build the package
colcon build --packages-select realsense_rgbd_streamer --symlink-install
source install/setup.bash

# Launch with RViz (includes pose overlay node)
ros2 launch realsense_rgbd_streamer viz_simple.launch.py

# Or run standalone
ros2 run realsense_rgbd_streamer pose_2d_overlay
```

## Node Details

**Name**: `pose_2d_overlay`

**Subscribes to**:
- `/realsense/camera_1/color/image_raw` (sensor_msgs/Image) - RGB camera feed
- `/uq/pose_2d` (uq_msgs/Pose2D) - 2D pose keypoints with uncertainty

**Publishes**:
- `/uq/pose_2d_overlay` (sensor_msgs/Image) - RGB image with pose drawn on it

**Publishing rate**: ~30 Hz

## Input Format

The `/uq/pose_2d` topic should publish a `uq_msgs/Pose2D` message with:
- `keypoints_2d` (float64[]): Flattened array `[x1, y1, x2, y2, ..., xn, yn]`
- `human_detected` (bool): If false, no overlay is drawn
- `n_joints` (int32): Number of joints (expected: 13)

Where each `(xi, yi)` pair represents the pixel coordinates of a keypoint in the image.

**Important**:
- Coordinates should be in pixels (not normalized)
- Invalid keypoints with `x <= 0` or `y <= 0` are automatically skipped
- The overlay is only drawn when `human_detected` is `true`
- The node expects exactly 13 joints

## Visualization

The node draws:
1. **Skeleton lines** (green) connecting related keypoints
2. **Keypoint circles** (red) at each joint location

## Skeleton Configuration

By default, the node uses a 13-joint pose model:
```
0: Nose
1: LShoulder         2: RShoulder
3: LElbow            4: RElbow
5: LWrist            6: RWrist
7: LHip              8: RHip
9: LKnee            10: RKnee
11: LAnkle          12: RAnkle
```

**Connections**:
- Nose to shoulders (0→1, 0→2)
- Left arm (1→3→5)
- Right arm (2→4→6)
- Torso (1↔2, 1→7, 2→8, 7↔8)
- Left leg (7→9→11)
- Right leg (8→10→12)

### Customizing Skeleton

To match a different pose estimation model, edit the `self.skeleton` list in `pose_2d_overlay.py`:

```python
# Example: Custom skeleton for different keypoint order
self.skeleton = [
    (0, 1),   # connection between keypoint 0 and 1
    (1, 2),   # connection between keypoint 1 and 2
    # ... add your connections
]
```

Each tuple `(i, j)` defines a line drawn between keypoint `i` and keypoint `j`.

## Customization Options

Edit `pose_2d_overlay.py` to customize:

**Colors**:
```python
# Line color (BGR format)
cv2.line(cv_image, pt1_int, pt2_int, (0, 255, 0), 2)  # Green lines

# Keypoint color (BGR format)
cv2.circle(cv_image, center, 4, (0, 0, 255), -1)  # Red circles
```

**Sizes**:
```python
# Line thickness
cv2.line(..., thickness=2)

# Circle radius
cv2.circle(..., radius=4, ...)
```

**Publishing rate**:
```python
# In __init__ method
self.timer = self.create_timer(0.033, self.publish_overlay)  # 30 Hz
# Change 0.033 to desired period in seconds
```

**Show keypoint labels**:
```python
# Uncomment in publish_overlay method
cv2.putText(cv_image, str(i), center,
           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
```

## Example: Testing with Dummy Data

```bash
# Publish test 2D pose data (13 joints for a standing person)
ros2 topic pub /uq/pose_2d uq_msgs/msg/Pose2D \
  "{header: {frame_id: 'camera_1_color_optical_frame'}, \
    keypoints_2d: [320, 180, 280, 220, 360, 220, 250, 280, 390, 280, 230, 320, 410, 320, 300, 380, 340, 380, 290, 450, 350, 450, 285, 520, 355, 520], \
    n_joints: 13, \
    human_detected: true, \
    is_ood: false, \
    ood_score: 0.0}" \
  --rate 10

# This publishes 13 keypoints forming a basic stick figure:
# Nose(320,180), LShoulder(280,220), RShoulder(360,220),
# LElbow(250,280), RElbow(390,280), LWrist(230,320), RWrist(410,320),
# LHip(300,380), RHip(340,380), LKnee(290,450), RKnee(350,450),
# LAnkle(285,520), RAnkle(355,520)
```

## Troubleshooting

**Overlay image not published**:
- Check both input topics are publishing
- Verify node is running: `ros2 node list`

**Skeleton connections look wrong**:
- Your pose model may use different keypoint ordering
- Modify `self.skeleton` to match your model's keypoint indices

**Missing keypoints**:
- Check if keypoint coordinates are positive (negative or zero are skipped)
- Verify keypoint data format is correct (flat array of x,y pairs)

**Performance issues**:
- Reduce publishing rate by increasing timer period
- Reduce line thickness and circle size
