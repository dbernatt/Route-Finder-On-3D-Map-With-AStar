from __future__ import print_function
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import math
import sys 


dirCol = [ 0,  1, 1, 1, 0, -1, -1, -1]
dirRow = [-1, -1, 0, 1, 1,  1,  0, -1]

offsetX = 0
offsetY = 0

m = 80
g = 10

class Point:
  def __init__(self, x, y, z,  blocked = 0, dist = -1, parent = None):
    self.x = int(x)
    self.y = int(y)
    self.z = z
    self.blocked = blocked
    self.dist = dist
    self.parent = parent
    self.f = 0
    self.g = 0
    self.h = 0

  def __eq__(self, other):
    return self.x == other.x and self.y == other.y

  def __str__(self):
    if self.parent is None:
      return "{ x = " + str(self.x) + ", y = " + str(self.y) + ", z = " + str(self.z) + ", blocked = " + str(self.blocked) + ", parent = None, dist = " + str(self.dist) + " }"
    else:
      return "{ x = " + str(self.x) + ", y = " + str(self.y) + ", z = " + str(self.z) + ", blocked = " + str(self.blocked) + ", parent = (" + str(self.parent.x)+ ", " + str(self.parent.y)  + "), dist = " + str(self.dist) + " }"
      

  def __repr__(self):
    if self.parent is None:
      return "{ x = " + str(self.x) + ", y = " + str(self.y) + ", z = " + str(self.z) + ", blocked = " + str(self.blocked) + ", parent = None, dist = " + str(self.dist) + " }"
    else:
      return "{ x = " + str(self.x) + ", y = " + str(self.y) + ", z = " + str(self.z) + ", blocked = " + str(self.blocked) + ", parent = (" + str(self.parent.x)+ ", " + str(self.parent.y)  + "), dist = " + str(self.dist) + " }"
      
def readSurface(filename):
  global offsetX, offsetY
  lines = []
  
  # Read lines from file
  with open(filename) as f:
    for line in f:
        line = line.split()
        x = int(float(line[0]))
        y = int(float(line[1]))
        z = float(float(line[2]))
        b = int(float(line[3]))
        lines.append((x, y, z, b))
  
  # Check if input is complete square matrix
  N = int(len(lines))
  root = math.sqrt(N)
  if int(root + 0.5) ** 2 != N:
    print("Not square matrix!")
    return None
  N = int(root)

  graph = []
  offsetX = lines[0][0]
  offsetY = lines[0][1]
  lineIndex = 0

  for _ in range(N):
    newRow = []
    for _ in range(N):
      newRow.append(Point(x = lines[lineIndex][0] - offsetX, y = lines[lineIndex][1] - offsetY, z = lines[lineIndex][2], blocked = lines[lineIndex][3]))
      lineIndex = lineIndex + 1
    graph.append(newRow)
  
  return graph

def readPoints(filename):
  global offsetX, offsetY

  with open(filename) as f:
    line = f.readline().split()
    start = (int(float(line[0])) - offsetX, int(float(line[1])) - offsetY, float(line[2]))
    line = f.readline().split()
    end = (int(float(line[0])) - offsetX, int(float(line[1])) - offsetY, float(line[2]))
  return start, end

def bfs(graph, start, end):
  n = len(graph)
  Q = []
  visited = []
  
  for _ in range(n):
    newRow = []
    for _ in range(n):
      newRow.append(False)
    visited.append(newRow)

  visited[start[1]][start[0]] = True
  graph[start[1]][start[0]].dist = 0

  Q.append(graph[start[1]][start[0]])

  while Q != []:
    current_point = Q.pop(0)
    for i in range(8):
      row = current_point.y + dirRow[i]
      col = current_point.x + dirCol[i]
      if row >= 0 and row < n and col >= 0 and col < n:

        if graph[row][col].blocked == 0 and not visited[row][col]:

          visited[row][col] = True
          graph[row][col].dist = graph[current_point.y][current_point.x].dist + 1
          graph[row][col].parent = graph[current_point.y][current_point.x]
          Q.append(graph[row][col])

          if(row == end[1] and col == end[0]):
            return graph
  return None

def minpath_bfs(graph, start, end):
  bfs(graph, start, end)
  path = []
  current = graph[end[1]][end[0]]
  while current is not None:
    path.append((current.x, current.y))
    current = current.parent

  return path[::-1] # Return reversed path

def astar(graph, start, end):
  # A* (star) Pathfinding
  
  # Initialize both open and closed list
  open_list = []
  closed_list = []
  n = len(graph)

  # Add the start node
  start_node = graph[start[1]][start[0]]
  end_node = graph[end[1]][end[0]]
  open_list.append(start_node)

  # Loop until you find the end
  while len(open_list) > 0:

    # Get the current node
    # let the currentNode equal the node with the least f value
    current_node = open_list[0]
    current_node_index = 0
    for index, item in enumerate(open_list):
      if item.f < current_node.f:
        current_node = item
        current_node_index = index
    
    # remove the currentNode from the openList
    open_list.pop(current_node_index)

    # add the currentNode to the closedList
    closed_list.append(current_node)
    # print(len(closed_list))

    # Found the goal
    if current_node == end_node:
      path = []
      current = current_node
      while current is not None:
        path.append((current.x, current.y))
        current = current.parent
      return path[::-1] # Return reversed path

    # Generate children
    # let the children of the currentNode equal the adjacent nodes
    childrens = []
    for i in range(8):
      row = current_node.y + dirRow[i]
      col = current_node.x + dirCol[i]
      if row >= 0 and row < n and col >= 0 and col < n and graph[row][col].blocked == 0:
        new_node = Point(col, row, graph[row][col].z, 0, -1, current_node)
        childrens.append(new_node)

    for child in childrens:

      # Child is on the closedList
      isIn = False
      for closed_child in closed_list:
        if child == closed_child:
          isIn = True
          break
      
      if isIn:
        continue


      # Create the f, g, and h values
      child.g = current_node.g + 1
      child.h = ((child.x - end_node.x) ** 2) + ((child.y - end_node.y) ** 2) + ((child.z - end_node.z) ** 2)
      child.f = child.g + child.h

      # Child is already in the open list
      isIn = False
      for open_node in open_list:
        if child == open_node and child.g > open_node.g:
          isIn = True
          break

      if isIn:
        continue
      # Add the child to the open list
      open_list.append(child)

def astar_energy(graph, start, end):
  global m
  global g
  # A* (star) Pathfinding
  
  # Initialize both open and closed list
  open_list = []
  closed_list = []
  n = len(graph)

  # Add the start node
  start_node = graph[start[1]][start[0]]
  end_node = graph[end[1]][end[0]]
  open_list.append(start_node)

  # Loop until you find the end
  while len(open_list) > 0:
    
    # print(len(closed_list))
    # Get the current node
    # let the currentNode equal the node with the least f value
    current_node = open_list[0]
    current_node_index = 0
    for index, item in enumerate(open_list):
      if item.f < current_node.f:
        current_node = item
        current_node_index = index
    
    # remove the currentNode from the openList
    open_list.pop(current_node_index)

    # add the currentNode to the closedList
    closed_list.append(current_node)
    # print(len(closed_list))

    # Found the goal
    if current_node == end_node:
      path = []
      current = current_node
      while current is not None:
        path.append((current.x, current.y))
        current = current.parent
      return path[::-1] # Return reversed path

    # Generate children
    # let the children of the currentNode equal the adjacent nodes
    childrens = []
    for i in range(8):
      row = current_node.y + dirRow[i]
      col = current_node.x + dirCol[i]
      if row >= 0 and row < n and col >= 0 and col < n and graph[row][col].blocked == 0:
        new_node = Point(col, row, graph[row][col].z, 0, -1, current_node)
        childrens.append(new_node)

    for child in childrens:

      # Child is on the closedList
      isIn = False
      for closed_child in closed_list:
        if child == closed_child:
          isIn = True
          break
      
      if isIn:
        continue
      
      child.g = current_node.g + 1
      child.h = ((child.x - end_node.x) ** 2) + ((child.y - end_node.y) ** 2) + ((child.z - end_node.z) ** 2)
      child.f = child.g + child.h + (child.z - current_node.z)
      d_current_child = math.sqrt(((current_node.x - child.x) ** 2) + ((current_node.y - child.y) ** 2) + ((current_node.z - child.z) ** 2))

      if current_node.z < child.z:
        c = Point(child.x, child.y, current_node.z, 0, -1, graph[current_node.y][current_node.x])
        d_child_c = math.sqrt(((child.z - c.z) ** 2))
        d_current_c = math.sqrt(((current_node.x - c.x) ** 2) + ((current_node.y - c.y) ** 2))
        sin_alfa = d_child_c / d_current_child
        cos_alfa = d_current_c / d_current_child
        F = m * g * sin_alfa
        s = d_child_c / sin_alfa
        W = F * s * cos_alfa
        child.f = child.f + W
      else:
        if child.z < current_node:
          c = Point(current_node.x, current_node.y, child.z, 0, -1, graph[current_node.y][current_node.x])
          d_child_c = math.sqrt(((child.x - c.x) ** 2) + ((child.y - c.y) ** 2))
          d_current_child = math.sqrt(((current_node.z - child.z) ** 2))
          sin_alfa = d_current_c / d_current_child
          cos_alfa = d_child_c / d_current_child
          F = m * g * sin_alfa
          s = d_current_c / sin_alfa
          W = F * s * cos_alfa
          child.f = child.f + W
        else:
          s = math.sqrt(((current_node.z - child.z) ** 2))
          F = m * g
          W = F * s
          child.f = child.f + W

      # Child is already in the open list
      isIn = False
      for open_node in open_list:
        if child == open_node and child.g > open_node.g:
          isIn = True
          break

      if isIn:
        continue
      # Add the child to the open list
      open_list.append(child)
    
def plotSurf(graph, start, end, path_bfs, path_astar, path_astar_energy):
  global offsetX, offsetY
  n = len(graph)
  x = []
  y = []
  z = []
  b = []

  for i in range(n):
    for j in range(n):
      x.append(graph[i][j].x + offsetX)
      y.append(graph[i][j].y + offsetY)
      z.append(graph[i][j].z)
      b.append(graph[i][j].blocked)

  start = np.array([start[0] + offsetX, start[1] + offsetY, start[2]])
  end = np.array([end[0] + offsetX, end[1] + offsetY, end[2]])

  x = np.array(x)
  y = np.array(y)
  z = np.array(z)

  fig = plt.figure()

  ax = fig.add_subplot(111, projection='3d')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')

  ax.text2D(0.05, 0.95, '(BLUE) BFS path length: %d' % len(path_bfs), transform=ax.transAxes)
  ax.text2D(0.05, 0.90, '(YELLOW) Astar path length: %d' % len(path_astar), transform=ax.transAxes)

  surf = ax.plot_trisurf(x,y,z, cmap=cm.coolwarm, linewidth=0, antialiased = True, alpha=.8)

  fig.colorbar(surf, shrink=0.5, aspect=5)

  ax.scatter(start[0], start[1], start[2], marker = "o", s = 50, c = "red", cmap = "brg")
  ax.scatter(end[0], end[1], end[2], marker = "o", s = 50, c = "red", cmap = "brg")

  for i in range(len(b)):
    if b[i] == 1:
      ax.scatter(x[i], y[i], z[i], marker = "o", s = 10, c = "black", cmap = "brg")

  # BFS path
  for i in path_bfs:
    row = int(i[1])
    col = int(i[0])
    ax.scatter(graph[row][col].x + offsetX, graph[row][col].y + offsetY,  graph[row][col].z, marker = "o", s = 20, c = "blue", cmap = "brg")
  
  # Astar path 
  for i in path_astar:
    row = int(i[1])
    col = int(i[0])
    ax.scatter(graph[row][col].x + offsetX, graph[row][col].y + offsetY,  graph[row][col].z, marker = "o", s = 20, c = "yellow", cmap = "brg")
  
  # Astar energy
  for i in path_astar_energy:
    row = int(i[1])
    col = int(i[0])
    ax.scatter(graph[row][col].x + offsetX, graph[row][col].y + offsetY,  graph[row][col].z, marker = "o", s = 20, c = "green", cmap = "brg")
  


  ax.view_init(30, 180)
      
  plt.show()

def main():
  graph = readSurface("surface.txt")
  start, end = readPoints("points.txt")

  path_bfs = minpath_bfs(graph, start, end)
#   print(path_bfs)
#   print(len(path_bfs))

  path_astar = astar(graph, start, end)
#   print(path_astar)
#   print(len(path_astar))

#   path_astar_energy = astar_energy(graph, start, end)
#   print(path_astar_energy)
#   print(len(path_astar_energy))

#   plotSurf(graph, start, end, path_bfs, path_astar, path_astar_energy)
  plotSurf(graph, start, end, path_bfs, path_astar, [])
#   plotSurf(graph, start, end, [], [], path_astar_energy)

if __name__ == '__main__':
  main()






