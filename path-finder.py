'''
Input format

Algorithm type
W H
X Y -> starting position 
		0 <= X <= W-1
		0 <= Y <= H -1
Max Height that wagon can climb 
N => Number of settling sites 
N lines => Coordinates of settling sites 
H lines => map of the terrain 


M >= 0, muddy land with height 0 and mud-level M 
M < 0, rock of height |M| with mud-level 0 
'''
from queue import PriorityQueue
from collections import deque
import os

moves_x = [-1,  0,  1, -1, 1, -1,  0,  1]
moves_y = [-1, -1, -1,  0, 0,  1,  1,  1]

num = 48
print("Test Case #", num)
INPUT_FILE = "HW1-Grading_Test_Solution_Cost/input/input" + str(num) + ".txt"
OUTPUT_FILE = "output" + str(num) + ".txt"

class Cell:

    def __init__(self, i, j, m):
        self.x = i
        self.y = j
        self.muddines = m
        self.rock_height = 0
        if m < 0:
            self.muddines = 0
            self.rock_height = abs(m)
        
    def get_coordinates(self):
        return (self.x, self.y)

    def __lt__(self, other):
        return False

def is_valid_move(current, neighbor):
    height_diff = abs(current.rock_height - neighbor.rock_height)
    return height_diff <= data.max_height
    
def is_valid_coordinates(x, y):
    if x<0 or y < 0:
        return False
    if x >=data.H or y >=data.W:
        return False
    return True

def heuristic(neighbor, settling_site):
    straight_move = 10
    diagonal_move = 14
    dx = abs(neighbor.x - settling_site.x)
    dy = abs(neighbor.y - settling_site.y)
    return straight_move * (dx + dy) + (diagonal_move - 2 * straight_move) * min(dx, dy)

def get_path(came_from, current):
    
    path = [current.get_coordinates()]
    # for key in came_from:
    #     print("came_From {} <- {}".format(key.get_coordinates(),came_from[key].get_coordinates()))
    while current in came_from:
        current = came_from[current]
        path.append(current.get_coordinates())
    # print("Path ", path)
    return path

def get_path_cost(path):
       
    # for key in came_from:
    #     print("came_From {} <- {}".format(key.get_coordinates(),came_from[key].get_coordinates()))
    # print(path)
    total_path_cost = 0
    for i in range(len(path)-2, -1, -1):
        prev = path[i+1]
        curr = path[i]
        tmp = abs(prev[0] - curr[0]) + abs(prev[1] - curr[1])
        if tmp == 2:
            total_path_cost += 14
        else:
            total_path_cost += 10
        if(data.algo == "A*"):
            total_path_cost += data.mesh_grid[curr[0]][curr[1]].muddines
            total_path_cost += abs(data.mesh_grid[curr[0]][curr[1]].rock_height - data.mesh_grid[prev[0]][prev[1]].rock_height)
    # print("Path ", path)
    return total_path_cost, len(path)

def runUCS(start, settling_site):
    pq = PriorityQueue()
    visited_hash = set()  
    count = 0
      # cost count(to maintain the insert order) start cell
    queue_set = {start} # keep track of items in the queue
    came_from = {}
    cost = {cell: float('inf') for i in data.mesh_grid for cell in i}
    cost[start] = 0
    pq.put((cost[start], count, start))
    while  not pq.empty() :
        current = pq.get()[2]
    #    queue_set.remove(current)
        if current == settling_site:
            path = get_path(came_from, current)
            return True, path
        
        if current not in visited_hash:
            # explore neighbors
            visited_hash.add(current)
            neighbors  = data.get_neighbours_with_cost(current)
            for neighbor, move_cost in neighbors:
                if (cost[current] + move_cost) < cost[neighbor] and neighbor not in visited_hash:
                    count +=1
                    came_from[neighbor] = current
                    cost[neighbor] = cost[current] + move_cost
                    pq.put((cost[neighbor], count, neighbor))
                    queue_set.add(neighbor)
    return False, []

def runASTAR(start, settling_site):
    pq = PriorityQueue()
    visited_hash = set()  
    count = 0
    pq.put((0, count, start))  # cost count(to maintain the insert order) start cell
    queue_set = {start}
    came_from = {}
    g_cost = {cell: float('inf') for i in data.mesh_grid for cell in i}
    f_cost = {cell: float('inf') for i in data.mesh_grid for cell in i}
    f_cost[start] = heuristic(start, settling_site)
    g_cost[start] = 0
    while not pq.empty() :
        current = pq.get()[2]
    #    queue_set.remove(current)
        if current == settling_site:
            path = get_path(came_from, current)
            return True, path
        
        if current not in visited_hash:
            # explore neighbors
            visited_hash.add(current)
            neighbors  = data.get_neighbours_with_cost(current)
            for neighbor, neighbor_g_cost in neighbors:
                tmp_g_cost = g_cost[current] + neighbor_g_cost
                if tmp_g_cost < g_cost[neighbor] and  neighbor not in visited_hash:
                    count +=1
                    came_from[neighbor] = current
                    g_cost[neighbor] = tmp_g_cost
                    f_cost[neighbor] = tmp_g_cost + heuristic(neighbor, settling_site)
                    pq.put((f_cost[neighbor], count, neighbor))
                    queue_set.add(neighbor)
    return False, []

def runBFS(start, settling_site):    
    # to track visited array 
    q = deque()
    visited_hash = set()  
    q.append(start)
    queue_set = {start} # keep track of items in the queue
    came_from = {}
    while  q :
        current = q.popleft()
        queue_set.remove(current)

        if current == settling_site:
            path = get_path(came_from, current)
            return True, path
       
        if current not in visited_hash:
            # explore neighbors
            visited_hash.add(current)
            neighbors  = data.get_neighbours(current)
            for neighbor in neighbors:
                if neighbor not in queue_set and neighbor not in visited_hash:
                    came_from[neighbor] = current
                    q.append(neighbor)
                    queue_set.add(neighbor)
    return False, []

# Store input.txt as InputData Class object
class InputData:
    def __init__(self):
        self.algo = ""
        self.W = 0
        self.H = 0
        self.start_coordinates =()
        self.max_height = 0
        self.num_of_setlling_sites = 0
        self.settling_sites = []
        

    def create_grid(self, grid):
        self.mesh_grid = []
        for i in range(self.H):
            self.mesh_grid.append([])
            for j in range(self.W):
                cell = Cell(i, j, grid[i][j])
                self.mesh_grid[i].append(cell)
    # 
    def get_neighbours(self, current): 
        neighbors = []
        for i in range(len(moves_x)):
            tmp_x = current.x + moves_x[i]
            tmp_y = current.y + moves_y[i]

            if is_valid_coordinates(tmp_x, tmp_y):
                neighbor = self.mesh_grid[tmp_x][tmp_y]
                if is_valid_move(current, neighbor):
                    neighbors.append(neighbor)
        return neighbors
    
    # Get neighbors with the uniform cost for USC algorithm 
    def get_neighbours_with_cost(self, current):
        neighbors = []
        for i in range(len(moves_x)):
            tmp_x = current.x + moves_x[i]
            tmp_y = current.y + moves_y[i]

            if is_valid_coordinates(tmp_x, tmp_y):
                neighbor = self.mesh_grid[tmp_x][tmp_y]
                if self.algo == "UCS" and is_valid_move(current, neighbor) :
                    neighbors.append((neighbor,self.get_move_cost(current, neighbor)))
                elif self.algo == "A*" and is_valid_move(current, neighbor):
                    neighbors.append((neighbor,self.get_neighbours_g_cost(current, neighbor)))
        return neighbors
    
    def get_neighbours_g_cost(self, current, neighbor):
        move = 0
        if abs(current.x - neighbor.x) == 0 or abs(current.y - neighbor.y) == 0:
            move += 10
        else:
            move += 14
        # mudlevel
        move += neighbor.muddines
        # height difference 
        move += abs(neighbor.rock_height - current.rock_height)
        return move

    def get_move_cost(self, current, neighbor):
        if abs(current.x - neighbor.x) == 0 or abs(current.y - neighbor.y) == 0:
            return 10
        return 14

    def print_grid(self):
        for i in range(self.H):
            for j in range(self.W):
                val = self.mesh_grid[i][j].muddines
                if self.mesh_grid[i][j].rock_height > 0:
                    val = -self.mesh_grid[i][j].rock_height
                print(" {:3} ".format(val), end= ",")
            print()

def driver():
    with open(INPUT_FILE, 'r') as f:
        # Algo 
        
        data.algo = f.readline().rstrip()
        #print("\nUsing Algorithm " + data.algo )

        # Dimensions of mesh grid 
        data.W, data.H = map(int, f.readline().rstrip().split())
        # print("Dimension of the grid {} x {}".format(data.W, data.H))

        # starting point 
        data.start_coordinates = tuple(map(int, f.readline().split()))

        # print("Starting point of our party {} , {}".format(data.start_coordinates[0], data.start_coordinates[1]))

        data.max_height = int(f.readline().rstrip())
        # print("Max height that wagon can climb is {}".format(data.max_height))

        data.num_of_setlling_sites = int(f.readline().rstrip())

        # print("Number of settling sites {}".format(data.num_of_setlling_sites))

        for i in range(data.num_of_setlling_sites):
            data.settling_sites.append(tuple(map(int, f.readline().rstrip().split())))
        #    print(data.settling_sites[-1])
        
        grid = []
        for i in range(data.H):
            grid.append(tuple(map(int, f.readline().rstrip().split())))
            # print(data.mesh_grid[-1])
        data.create_grid(grid)
        #data.print_grid()
        f.close()
    
    if os.path.isfile(OUTPUT_FILE):
        f = open(OUTPUT_FILE, 'w')
        f.truncate()
    else:
        f = open(OUTPUT_FILE, "w")
    
    start = data.mesh_grid[data.start_coordinates[1]][data.start_coordinates[0]]
        
    if data.algo == "BFS":
        
        for i, site  in enumerate(data.settling_sites):
            #print(site)
            res, path = runBFS(start, data.mesh_grid[site[1]][site[0]])
            if res:
                total_cost, path_length = get_path_cost(path)
                print("Path length: ",path_length-1, " Total Path cost : ", path_length-1)
                line = ""
                for t in range(len(path)-1, -1, -1):
                    line = line + "{},{} ".format(path[t][1],path[t][0])
                line = line.rstrip()
                line += "\n"
                f.write(line)
            else:
                print("FAIL")
                f.write("FAIL\n")
        f.close()

    elif data.algo == "UCS":
        for i, site  in enumerate(data.settling_sites):
            #print(site)
            res, path = runUCS(start, data.mesh_grid[site[1]][site[0]])
            if res:
                total_cost, path_length = get_path_cost(path)
                print("Path length: ",path_length-1 , " Total Path cost : ", total_cost)
                line = ""
                for t in range(len(path)-1, -1, -1):
                    line = line + "{},{} ".format(path[t][1],path[t][0])
                line = line.rstrip()
                line += "\n"
                f.write(line)
            else:
                print("FAIL")
                f.write("FAIL\n")
        f.close()

    elif data.algo == "A*":
        for i, site  in enumerate(data.settling_sites):
            #print(site)
            res, path = runASTAR(start, data.mesh_grid[site[1]][site[0]])
            if res:
                total_cost, path_length = get_path_cost(path)
                print("Path length: ",path_length-1, " Total Path cost : ", total_cost)
                line = ""
                for t in range(len(path)-1, -1, -1):
                    line = line + "{},{} ".format(path[t][1],path[t][0])
                line = line.rstrip()
                line += "\n"
                f.write(line)
            else:
                print("FAIL")
                f.write("FAIL\n")
        f.close()
data = InputData()
driver()