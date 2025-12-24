# Robot Lab 6 – Click-to-Move Autonomous Navigation

**Team Members:** Alex Anderson, Trenton Pham, Rafael Copado  
**Course:** CS 425 Robotics  

This project implements click-based autonomous navigation, allowing a user to select a
destination directly on the robot’s camera feed. The robot then autonomously drives
toward the clicked location using vision-based localization and distance comparisons.

---

## Project Overview

Unlike earlier labs that relied on tracking colored objects, this lab focuses on
**user-directed navigation**. A mouse click on the camera image defines a goal point,
and the robot continuously evaluates whether its movement is reducing the distance
to that target.

This approach demonstrates how perception, user interaction, and control logic can
be combined to achieve autonomous navigation without requiring full geometric modeling
or camera calibration.

---

## System Design

### User Interaction
- A mouse callback captures pixel coordinates when the user clicks on the camera feed
- A circle is drawn at the clicked location to visually confirm the target

### Robot Localization
- A binary mask and connected components analysis identify the robot in the image
- The largest detected blob is selected as the robot
- Pixel coordinates (`me_x`, `me_y`) approximate the robot’s position

### Navigation Logic
- Pixel distance to the target is computed continuously
- Motion decisions are based on distance trends:
  - If driving forward decreases distance → continue
  - If distance increases → reverse
  - If neither improves distance → spin to reorient
- The robot stops once it is within a fixed threshold distance of the target

This results in a simple but effective vision-based navigation strategy.

---

## Results

The robot consistently:
- Drove toward the clicked target location
- Adjusted motion when forward movement increased distance
- Reoriented itself when necessary
- Stopped automatically once close to the goal

Despite relying only on pixel-distance trends, the behavior was repeatable and robust
under stable lighting conditions.

---

## Demo Video

🎥 **Click-to-move autonomous navigation demo:**  
https://youtube.com/shorts/RLCkYp2YeCI?si=F7wmvosarY4w6lIc

---

## Reflection

This lab highlighted how interactive input can be combined with perception and control
to produce autonomous navigation. Even without camera calibration or angle estimation,
the robot was able to reach user-defined targets using simple distance-based logic.

The primary challenge was maintaining reliable robot localization under varying lighting
conditions and ensuring the mask remained stable.

---

## Future Improvements

- Use centroid-based localization instead of bounding-box coordinates
- Convert pixel distance to real-world distance using camera calibration
- Add proportional steering based on horizontal offset
- Support multiple sequential waypoints
- Improve robustness under changing lighting conditions

---

## Files

- `lab6Robot.py` — Click-based autonomous navigation using OpenCV and distance logic
