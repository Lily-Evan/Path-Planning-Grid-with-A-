 A* Path Planning Visualization

This project implements the **A\*** algorithm on a 2D grid with obstacles and visualizes the search process step by step using animation.  
The goal is to find the shortest path from a starting point (Start) to a target point (Goal).

## 🔹 Features
- 20x20 grid with predefined obstacles.
- **A\*** algorithm using Manhattan distance as the heuristic.
- Visualization with **matplotlib.animation**:
  - Blue circle = Start
  - Red X = Goal
  - Black cells = Obstacles
  - Yellow = Open set (nodes being explored)
  - Cyan = Closed set (nodes already visited)
  - Green line = Final path

## 🔧 Requirements
- Python 3.8+
- Libraries:
  ```bash
  pip install matplotlib numpy
▶️ Run
Execute:

bash
Αντιγραφή κώδικα
python astar_path_planning.py
An animation window will appear, showing how the A* algorithm explores the grid and finds the path.

💾 Save as Video (Optional)
To save the animation as an MP4 video:

Install ffmpeg:

Linux: sudo apt install ffmpeg

Windows/Mac: Download here and add it to your PATH

Add the following lines to the script:

python
Αντιγραφή κώδικα
from matplotlib.animation import FFMpegWriter
writer = FFMpegWriter(fps=5, bitrate=1800)
ani.save("astar_path_planning.mp4", writer=writer)
📚 References
A* Search Algorithm

Matplotlib Animation Documentation
