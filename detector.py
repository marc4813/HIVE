import cv2;
import numpy as np;
import math;

def inBound(y, x, yLim, xLim):
  return y < yLim and x < xLim

def countPixels(image, start, directions, used):
  yLim, xLim = image.shape
  queue = [start]
  res = 0
  shape = []

  while(queue):
    y, x = queue.pop(0)

    if(not used[y][x]):
      dist = 0
      used[y][x] = True
      res+=1
      shape.append((y, x))

      for dY, dX in directions:
        newY = y + dY
        newX = x + dX

        if(inBound(newY, newX, yLim, xLim)):
          if(image[newY][newX] == 0):
            queue.append((newY, newX))

  return (res, shape)

def markUsed(image, start, directions, used):
  yLim, xLim = image.shape
  queue = [start]

  while(queue):
    y, x = queue.pop(0)

    if(not used[y][x]):
      used[y][x] = True

      for dY, dX in directions:
        newY = y + dY
        newX = x + dX

        if(inBound(newY, newX, yLim, xLim)):
          if(image[newY][newX] == 0):
            queue.append((newY, newX))

def isNormal(temp):
  res = False
  a = temp[0]
  b = temp[1]
  c = temp[2]
  d = temp[3]

  yA, xA = a
  yB, xB = b
  yC, xC = c
  yD, xD = d

  if(yA == yB and xA != xB):
    if(yB != yC and xB == xC):
      if(yC == yD and xC != xD):
        if(yD != yA and xD == xA):
          res = True

  return res

def equalSides(temp):
  a = temp[0]
  b = temp[1]
  c = temp[2]
  d = temp[3]
  distAB = math.dist(a, b)
  distCD = math.dist(c, d)
  distBC = math.dist(b, c)
  distDA = math.dist(d, a)

  return distAB == distCD and distBC == distDA

def isNormal(temp):
  res = False
  a = temp[0]
  b = temp[1]
  c = temp[2]
  d = temp[3]

  yA, xA = a
  yB, xB = b
  yC, xC = c
  yD, xD = d

  if(yA == yB and xA != xB):
    if(yB != yC and xB == xC):
      if(yC == yD and xC != xD):
        if(yD != yA and xD == xA):
          res = True

  return res and equalSides(temp)

def isLower(a, b, c, d):
  res = False
  yA, xA = a
  yB, xB = b
  yC, xC = c
  yD, xD = d

  if(yA < yB and xA < xB):
    if(yB < yC and xB > xC):
      if(yC > yD and xC > xD):
        if(yD > yA and xD < xA):
          res = True

  return res

def isUpper(a, b, c, d):
  res = False
  yA, xA = a
  yB, xB = b
  yC, xC = c
  yD, xD = d

  if(yA > yB and xA < xB):
    if(yB < yC and xB < xC):
      if(yC < yD and xC > xD):
        if(yD > yA and xD > xA):
          res = True

  return res

def isSlanted(temp):
  a = temp[0]
  b = temp[1]
  c = temp[2]
  d = temp[3]
  res = isUpper(a, b, c, d) or isLower(a, b, c, d)

  return res and equalSides(temp)

def perm(shapes, currLoc, nShapes, used, temp, permBuffer):
  if(currLoc == 4):
    if(isNormal(temp) or isSlanted(temp)):
      permBuffer.append(temp.copy())

    return

  for next in range(nShapes):
    if(not used[next]):
      dist = 0

      if(currLoc != 0):
        dist = math.dist(shapes[next], temp[currLoc-1]);

      if(currLoc == 0 or (dist > 4.9 and dist < 10)):
        used[next] = True
        temp.append(shapes[next])
        perm(shapes, currLoc+1, nShapes, used, temp, permBuffer);
        temp.pop(currLoc)
        used[next] = False

def comp(perm):
  a, b, c, d = tuple(perm)

  distAB = math.dist(a, b)
  distBC = math.dist(b, c)
  distCD = math.dist(c, d)

  return (distAB, distBC, distCD)
    
def findPayloads(grid, height, width):
  image = np.empty((height, width))
  directions = []
  coords = []
  possibleLocations = []
  finalShapes = []
  midpoints = []
  used = np.zeros((height, width))
  dX = [-1, 0, 0, 1]
  dY = [0, -1, 1, 0]

  np.fill(image, 255)

  for y in range(height):
    for x in range(width):
      if(grid[y][x] >= 90):
        image[y][x] = 0

  for dx in dX:
    for dy in dY:
      directions.append((dy, dx))

  for y in range(height):
    for x in range(width):
      if(image[y][x] == 0):
          possibleLocations.append((y, x))

  for loc in possibleLocations:
    res, shape = countPixels(image, loc, directions, used)

    if(res >= 1 and res <= 5):
      for pair in shape:
          coords.append(pair)

  nShapes = len(coords)
  used = np.zeros(nShapes)
  
  perm(coords, 0, nShapes, used, [], finalShapes)

  used = np.zeros((height, width))
  nShapes = len(finalShapes)
  
  finalShapes.sort(key = comp, reverse = True)

  for next in range(nShapes):
    nextShape = finalShapes[next]
    canAdd = True

    for coord in nextShape:
      if(used[coord[0]][coord[1]]):
        canAdd = False
        break

    if(canAdd):
      markUsed(image, coord, directions, used)
      
      midY = math.floor((nextShape[1][0] + nextShape[2][0]) / 2)
      midX = math.floor((nextShape[0][1] + nextShape[1][1]) / 2)

      midpoints.append((midY, midX))

  return midpoints