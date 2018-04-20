## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation `plan_path()` that includes...
* setting global_home by the latitue and longitude from the first line of the colliders.cvs.
* reading obstacles from the file.
    * Obstacles have the local position from the global_home in that file.
* making grid from the obstacles.
    * The local position of grid(0, 0) is (north_min, east_min).
* setting grid_start and grid_goal.
* finding path from A* through the grid.
* sending waypoints by the path.
This scripts make the drone fly zig-zag along the waypoints. It can be smoothed out by pruning the path.


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
The csv file describes the downtown San Francisco environment.
![Map of SF](./misc/map.png)

The first line of the csv file is lat0 and lon0 of the certain position which the local position of obstacles is relatvie to.
```
# DONE: read lat0, lon0 from colliders into floating point values
with open('colliders.csv') as f:
    home_pos_data = f.readline().split(',')
lat0 = float(home_pos_data[0].strip().split(' ')[1])
lon0 = float(home_pos_data[1].strip().split(' ')[1])

# DONE: set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
The current local position can be obtained from,
```
current_global_position = (self._longitude, self._latitude, self._altitude)
current_local_postion = global_to_local(current_global_position, self.global_home)
```
This value is a little different with self.local_position from the same global_position. It seems that self.local_position is updated a little latter.

#### 3. Set grid start position from local position
I set the grid_start position to the current position as following.
```
grid_start = (int(current_local_position[0] -north_offset), int(current_local_position[1] -east_offset))
```
Or when I use the graph representation, I use this,
```
closest_grid_start = closest_point(G, grid_start)
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. (lat, lon) within the map is rendered to a goal location on the grid.
```
global_goal = (-122.39827335, 37.79639627, 0)
local_goal = global_to_local(global_goal, self.global_home)
grid_goal = (int(local_goal[0] - north_offset), int(local_goal[1] - east_offset)) #(750., 370.) 
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I modified the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2)
```
    NORTH_EAST = (-1, 1, np.sqrt(2))
    NORTH_WEST = (-1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
```
And the actions should be checked whether it's available in each cell as following
```
if x - 1 < 0 or grid[x - 1, y] == 1:
    valid_actions.remove(Action.NORTH)
if x + 1 > n or grid[x + 1, y] == 1:
    valid_actions.remove(Action.SOUTH)
if y - 1 < 0 or grid[x, y - 1] == 1:
    valid_actions.remove(Action.WEST)
if y + 1 > m or grid[x, y + 1] == 1:
    valid_actions.remove(Action.EAST)

if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if x - 1 < 0 or y + 1 < 0 or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 < 0 or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x + 1 < 0 or y + 1 < 0 or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```
A* search for grid chooses the next_node by applying these valid_actions.
```
for action in valid_actions(grid, current_node):
    # get the tuple representation
    da = action.delta
    next_node = (current_node[0] + da[0], current_node[1] + da[1])
    branch_cost = current_cost + action.cost
    queue_cost = branch_cost + h(next_node, goal)
```
I also tried the graph representaion. The graph is made by Voronoi algorithm with `the center points of the obstacles` and each edge from graph.ridge_vertices is checked for collision with `the grid of obstacles`.
```
edges = create_edges_from_grid(grid, points)
```
In case of the graph represetation, a* search can choose the next_node from the graph directly.
```
for next_node in graph[current_node]:
    cost = graph.edges[current_node, next_node]['weight']
    branch_cost = current_cost + cost
    queue_cost = branch_cost + h(next_node, goal)
```
Lastly, I applied A* search to grid from the medial axis.
```
skeleton = medial_axis(invert(grid))
closest_grid_start, closest_grid_goal = find_start_goal(skeleton, grid_start, grid_goal) 
path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(closest_grid_start), tuple(closest_grid_goal))
```

#### 6. Cull waypoints 
I use collinearity check to cull waypoints.
If the 3 points are in a line, remove the 2nd point and the 3rd point now becomes 2nd point and the check is redone with a new third point on the next iteration.
```
pruned_path = [p for p in path]
i = 0
pruned_count = 0
while i < len(pruned_path) - 2:
    p1 = point(pruned_path[i])
    p2 = point(pruned_path[i+1])
    p3 = point(pruned_path[i+2])
    
    if collinearity_check(p1, p2, p3):
        pruned_path.remove(pruned_path[i+1])
        pruned_count += 1
    else:
        i += 1
```


### Execute the flight
#### 1. Does it work?
It works!
![result](./misc/result.png)
### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


