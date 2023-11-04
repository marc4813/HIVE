import cv2;
import numpy as np;
import math;

def inBound(y, x, yLim, xLim):
  return y < yLim and x < xLim;

def countPixels(image, start, directions, used):
  yLim, xLim = image.shape;
  queue = [start];
  res = [];

  while(queue):
    y, x = queue.pop(0);

    if(not used[y][x]):
      used[y][x] = True;
      
      res.append((y, x));

      for dY, dX in directions:
        newY = y + dY;
        newX = x + dX;

        if(inBound(newY, newX, yLim, xLim)):
          if(image[newY][newX] == 0):
            queue.append((newY, newX));
          else:
            image[newY][newX] = 255;

  return res;

def clear(image, start, directions):
  yLim, xLim = image.shape;
  queue = [start];
  used = np.zeros((yLim, xLim));

  while(queue):
    y, x = queue.pop(0);

    if(not used[y][x]):
      used[y][x] = True;

      for dY, dX in directions:
        newY = y + dY;
        newX = x + dX;

        if(inBound(newY, newX, yLim, xLim) and image[newY][newX] != 1):
          if(image[newY][newX] != 0):
            queue.append((newY, newX));
          else:
            image[newY][newX] = 255;

def main():
  image = cv2.imread("map.png");
  image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY);
  height, width = image.shape;
  directions = [];
  shapes = [];
  finalSet = [];
  locations = [];
  used = np.zeros((height, width));
  dX = [-1, 0, 0, 1];
  dY = [0, -1, 1, 0];

  for dx in dX:
    for dy in dY:
      directions.append((dy, dx));

  for y in range(height):
    for x in range(width):
      if(image[y][x] < 254):
        res = countPixels(image, (y, x), directions, used);
        length = len(res);

        if(length >= 9 and length <= 30):
          shapes.append(res);

  nShapes = len(shapes);

  while(nShapes >= 4):
    coords = shapes.pop(0);
    distances = [];
    a = coords[0];
    nShapes-=1;
    
    for shape in range(nShapes):
      distance = math.dist(a, shapes[shape][0]);
      distances.append((shapes[shape][0], distance, shape));

    distances.sort(key = lambda comp : (comp[1]));

    b = distances[0];
    c = distances[1];
    d = distances[2];

    for index in [b[2], c[2]-1, d[2]-2]:
      shapes.pop(index);

    finalSet.append([a, b[0], c[0], d[0]]);

    nShapes-=3;

  for coords in finalSet:
    topLeft = (coords[0][1], coords[0][0]);
    bottomRight = (coords[3][1], coords[3][0]);
    midX = math.floor((bottomRight[0] - topLeft[0]) / 2) + topLeft[0];
    midY = math.floor((bottomRight[1] - topLeft[1]) / 2) + topLeft[1];
    image = cv2.rectangle(image, topLeft, bottomRight, 1, 1);
    
    clear(image, (midY, midX), directions);

    image[midY][midX] = 0;
    
    locations.append((midY, midX));

  cv2_imshow(image);

main();