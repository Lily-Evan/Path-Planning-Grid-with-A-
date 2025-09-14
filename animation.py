import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import heapq

# --- Grid Setup ---
rows, cols = 20, 20
grid = np.zeros((rows, cols))

# Προσθήκη εμποδίων
obstacles = [(5, y) for y in range(3, 15)] + [(10, y) for y in range(5, 18)]
for (x, y) in obstacles:
    grid[x, y] = 1  # 1 = εμπόδιο

start = (0, 0)
goal = (19, 19)

# --- A* Algorithm ---
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

def astar_with_steps(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    steps = []  # αποθήκευση βημάτων

    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        steps.append(("current", current))

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            steps.append(("path", data))
            return steps

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < grid.shape[0]:
                if 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                steps.append(("open", neighbor))
    
    return steps

# --- Εκτέλεση ---
steps = astar_with_steps(grid, start, goal)

# --- Οπτικοποίηση με Animation ---
fig, ax = plt.subplots()
ax.imshow(grid.T, cmap="gray_r", origin="lower")

# Σημάδια Start & Goal
ax.scatter(start[0], start[1], marker="o", color="blue", s=100, label="Start")
ax.scatter(goal[0], goal[1], marker="X", color="red", s=100, label="Goal")

open_points, = ax.plot([], [], "yo", markersize=6, label="Open Set")
closed_points, = ax.plot([], [], "co", markersize=6, label="Closed Set")
path_line, = ax.plot([], [], "g-", linewidth=2, label="Path")

def update(frame):
    action, cell = steps[frame]
    if action == "open":
        xdata, ydata = open_points.get_data()
        open_points.set_data(np.append(xdata, cell[0]), np.append(ydata, cell[1]))
    elif action == "current":
        xdata, ydata = closed_points.get_data()
        closed_points.set_data(np.append(xdata, cell[0]), np.append(ydata, cell[1]))
    elif action == "path":
        px, py = zip(*cell)
        path_line.set_data(px, py)
    return open_points, closed_points, path_line

ani = animation.FuncAnimation(fig, update, frames=len(steps), interval=200, blit=True, repeat=False)

plt.legend()
plt.title("A* Path Planning (Step by Step)")
plt.show()
