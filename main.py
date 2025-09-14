import matplotlib.pyplot as plt
import numpy as np
import heapq

# --- Ορισμός Grid ---
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
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan Distance

def astar(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

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
    
    return False

# --- Εκτέλεση ---
path = astar(grid, start, goal)

# --- Οπτικοποίηση ---
plt.imshow(grid.T, cmap="gray_r", origin="lower")

# Start - Goal
plt.scatter(start[0], start[1], marker="o", color="blue", s=100, label="Start")
plt.scatter(goal[0], goal[1], marker="X", color="red", s=100, label="Goal")

# Path
if path:
    px, py = zip(*path)
    plt.plot(px, py, color="green", linewidth=2, label="Path")
else:
    print("No path found!")

plt.legend()
plt.title("A* Path Planning")
plt.show()
