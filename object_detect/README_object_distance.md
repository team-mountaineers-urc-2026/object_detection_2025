# Object Distance Node

## object_distance.py
Subscribes to the YOLO node and publishes the estimated position of detected objects.

---

## Running the Node

### 1. Camera Manager

Make sure the Camera Manager package is installed and sourced:

https://github.com/wvu-urc/CameraManager/tree/main

Start the autonomy camera launch file (with your workspace sourced):

```
ros2 launch CameraManager cm_autonomy.launch.py
```

---

### 2. Object Detection

In a new terminal (with your workspace sourced), run:

```
ros2 run object_detect object_distance
```

You should now see object detection data and confidence values being printed in the terminal.

Hold one of the mission objects in front of the camera. Detections should begin appearing.

---

## Changing the Confidence Threshold

YOLO detection confidence is controlled by a Float32 subscriber on the topic:

/confidence_changer

- Default threshold: 0.3
- Acceptable range: 0.0 to 1.0

To update the confidence threshold while the node is running, use:

```
ros2 topic pub --once /confidence_changer std_msgs/msg/Float32 "{data: 0.5}"
```

Replace 0.5 with any desired value between 0.0 and 1.0.

Values outside this range will have no effect since confidence is normalized.

---

## Notes

- Make sure all workspaces are properly sourced before running commands.
- Ensure the YOLO node is running before attempting to adjust the confidence.
- Use `ros2 topic list` then `ros topic echo topic_name` to verify messages are being published.
