import numpy as np
import cv2
import random
from queue import PriorityQueue
from ament_index_python.packages import get_package_share_directory
import os

 # Define Colors
FREE=(255,255,255)
OBSTACLE=(0,0,0)
CLEAR= (12, 253, 240)
GRID=(233, 226, 230, 0.2)
START=(132, 222, 15, 0.8)
GOAL=(254, 0, 0, 0.8)
EXPLORED= (64, 188, 237)
TRACK= (16, 141, 16)
PATH= (16, 16, 16)
ROBOT= (62, 169, 119)
ROBOT_CLEAR= (255, 254, 255)
        
class Node:
    node_id = 0
    def __init__(self, state, parent=None):
        self.x = state[0]
        self.y = state[1]
        self.parent_id = None
        self.id = Node.node_id+1
        Node.node_id += 1
        self.cost = 0
        self.cost_to_go = 0
        self.cost_to_come = 0

 
    
    

    def __lt__(self, other):
        return self.cost < other.cost
    

class Astar:
    
    def __init__(self) -> None:
        
        # self.start = Node((2050,1660))
        # self.goal = Node((1034,654))
        
        self.start = None
        self.goal = None
        
        
        self.step_size =50
        
        self.OBJ = 10
        self.ROBOT = 10
        
        self.WIDTH = 2076
        self.HEIGHT = 1724
        self.scale_x = int(self.WIDTH / 2)
        self.scale_y = int(self.HEIGHT / 2)
        self.v = 50
        self.map = np.ones((self.HEIGHT,self.WIDTH, 3), dtype=np.uint8) * 255
        
        self.visited = np.zeros((self.WIDTH//self.v+1, self.HEIGHT//self.v+1), dtype=np.uint8)
        print(f"Visited shape: {self.visited.shape}")
        self.create_map()
        
        self.open_list = PriorityQueue()
        self.closed_list = set()
        
        
        
        
    def create_map(self):
        # Create a map of the environment with the obstacles
        rectangles = [(84,74,466,226),(636,74,366,226),(1084,74,366,226),(1534,74,466,226),
                        (84,374,466,526),(636,374,366,526),(1084,374,366,526),(1534,374,466,526),
                        (84,974,466,576),(636,974,366,576),(1084,974,366,576),(1534,974,466,576),
                        (84,1624,466,26),(636,1624,366,26),(1084,1624,366,26),(1534,1624,466,26)]
        
        for rect in rectangles:
            # print(f"Rectangle {rect} added")
            x,y,size_x,size_y=rect[0],self.HEIGHT-rect[3]-rect[1],rect[2],rect[3]
            cv2.rectangle(self.map, (x-self.OBJ-self.ROBOT, y-self.OBJ-self.ROBOT), (x +self.OBJ+self.ROBOT+ size_x, y +self.OBJ+self.ROBOT +size_y), ROBOT_CLEAR, thickness=-1)
            cv2.rectangle(self.map, (x-self.OBJ, y-self.OBJ), (x +self.OBJ+ size_x, y +self.OBJ+ size_y), CLEAR, thickness=-1)
            cv2.rectangle(self.map, (x, y), (x + size_x, y + size_y), OBSTACLE, thickness=-1)
        
        print("Select the start and goal points")
        new_size = (self.scale_x, self.scale_y)
        resized_image = cv2.resize(self.map, new_size, interpolation=cv2.INTER_AREA)

        # Display the image
        cv2.imshow("Rectangles", resized_image)
        # Set the mouse callback function for the image window
        cv2.setMouseCallback("Rectangles", self.mouse_callback)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    def mouse_callback(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            
            x,y = int(2*x), int(2*y)
            print(f"Mouse clicked at ({x}, {y})")
            if self.is_obstacle((x, y)):
                print("Selected point is an obstacle. Please select another point.")
                return
            if self.start is None:
                print("Start point selected")
                self.start = Node((x, y))
                cv2.circle(self.map, (x, y), 5, START, -1)
            elif self.goal is None:
                print("Goal point selected")
                self.goal = Node((x, y))
                cv2.circle(self.map, (x, y), 5, GOAL, -1)
            print(f"Color at point ({x}, {y}): {self.map[y, x]}")
            
    def is_obstacle(self, point):
        # Check if the given point is inside an obstacle
        x, y = int(point[0]),int(point[1])
        if x < 0 or x >= self.WIDTH or y < 0 or y >= self.HEIGHT:
            return True
        
        if (self.map[y, x] == OBSTACLE).all() or (self.map[y, x] == CLEAR).all() or (self.map[y, x] == ROBOT_CLEAR).all():
            return True
        else:
            return False
        
    def plan(self):
        print("Planning path")
        print(f"Start: {self.start.x}, {self.start.y}")
        print(f"Goal: {self.goal.x}, {self.goal.y}")
        
        self.open_list.put(self.start)
        self.visited[self.start.x//self.v, self.start.y//self.v] = 1
        
        
        while not self.open_list.empty():
            current = self.open_list.get()
            self.closed_list.add(current)
            if self.check_goal(current):
                print("Goal reached")
                self.goal = current
                cv2.destroyAllWindows()
                return True
            
            
            
            for neighbour in self.get_neighbours(current):                
                if neighbour not in self.open_list.queue:
                    neighbour.cost = current.cost_to_come+self.cost_to_go(neighbour)+self.step_size
                    neighbour.cost_to_come = current.cost_to_come + self.step_size
                    neighbour.cost_to_go = self.cost_to_go(neighbour)
                    neighbour.parent_id = current.id
                    self.open_list.put(neighbour)
                    self.add_to_map(current, neighbour)
           
        return False
    
    def save_path_to_file(self, path, filepath):
        try:
            with open(filepath, 'w') as file:
                for point in path:
                    file.write(f"{point[0]}, {point[1]}\n")
        except FileNotFoundError:
            with open(filepath, 'w+') as file:
                for point in path:
                    file.write(f"{point[0]}, {point[1]}\n")
                    
    def path_to_goal(self):
        # Trace back from the goal node to the start node to generate the final path
        path = []
        current_node = self.goal
        print(f"Goal: {current_node.x, current_node.y}")
        while current_node.parent_id is not None:
            for node in self.closed_list:
                if node.id == current_node.parent_id:
                    path.append((current_node.x,current_node.y))
                    current_node = node
                    continue
        path.append((self.start.x, self.start.y))
        path.reverse()
        
        return path
    
    def cost_to_go(self, node):
        # Calculate the cost of moving from 'node' to the goal
        return self.distance(node, self.goal)
    
    def distance(self, node1, node2):
        return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    
    def check_goal(self, node):
        if self.distance(node, self.goal) < 50:
            return True
        return False
    
    def add_to_map(self, from_node, to_node):
        
        self.map = cv2.line(self.map, (from_node.x,from_node.y), (to_node.x,to_node.y), EXPLORED, 1)
        self.map = cv2.circle(self.map, (to_node.x,to_node.y), 1, TRACK, -1)
        resized_map = cv2.resize(self.map, (self.scale_x, self.scale_y), interpolation=cv2.INTER_AREA)
        cv2.imshow("Astar Map", resized_map)
        cv2.waitKey(1)
        
    def show_map(self):
        new_size = (self.scale_x, self.scale_y)
        resized_image = cv2.resize(self.map, new_size, interpolation=cv2.INTER_AREA)
        cv2.imshow("Astar Map", resized_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        
    def get_neighbours(self, node):
        neighbors = []
        for angle in range(0, 360, 30):  # Generate neighbors in 45-degree increments
            radian = np.radians(angle)
            dx = self.step_size * np.cos(radian)
            dy = self.step_size * np.sin(radian)
            new_state = (int(node.x + dx), int(node.y + dy))
            print(f"New state: {new_state}")
            print(f"Index {new_state[0]//self.v, new_state[1]//self.v}")
            
            if self.is_obstacle(new_state):
                continue
            
            if (self.visited[new_state[0]//self.v, new_state[1]//self.v] == 1):
                continue
            self.visited[new_state[0]//self.v, new_state[1]//self.v] = 1
            neighbor_state = Node(new_state)
            neighbor_state.cost_to_come = node.cost_to_come + self.step_size
            neighbors.append(neighbor_state)
        
        return neighbors
        


if __name__ == "__main__":
    astar = Astar()
    if astar.plan():
        path = astar.path_to_goal()
        if path is not None:
            print(f"Path found: {path}")
            for i in range(len(path) - 1):
                start_point = (int(path[i][0]), int(path[i][1]))
                end_point = (int(path[i+1][0]), int(path[i+1][1]))
                astar.map = cv2.line(astar.map, start_point, end_point, PATH, 2)
        resized_map = cv2.resize(astar.map, (astar.scale_x, astar.scale_y), interpolation=cv2.INTER_AREA)
        cv2.imshow("Path", resized_map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()        

     
        
        