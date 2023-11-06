import cv2;
import numpy as np;
import math;
from google.colab.patches import cv2_imshow;

def inBound(y, x, yLim, xLim):
  return y < yLim and x < xLim;

def countPixels(image, start, directions, used):
  yLim, xLim = image.shape;
  queue = [start];
  res = 0;
  shape = [];

  while(queue):
    y, x = queue.pop(0);

    if(not used[y][x]):
      dist = 0;
      used[y][x] = True;
      res+=1;
      shape.append((y, x));

      for dY, dX in directions:
        newY = y + dY;
        newX = x + dX;

        if(inBound(newY, newX, yLim, xLim)):
          if(image[newY][newX] == 0):
            queue.append((newY, newX));

  return (res, shape);

def perm(shapes, currLoc, nShapes, used, temp, permBuffer):
  if(currLoc == 4):
    a = temp[0];
    b = temp[1];
    c = temp[2];
    d = temp[3];

    yA, xA = a;
    yB, xB = b;
    yC, xC = c;
    yD, xD = d;

    if(yA == yB and xA != xB):
      if(yB != yC and xB == xC):
        if(yC == yD and xC != xD):
          if(yD != yA and xD == xA):
            permBuffer.append(temp.copy());
    return;

  for next in range(nShapes):
    if(not used[next]):
      dist = 0;

      if(currLoc != 0):
        dist = math.dist(shapes[next], temp[currLoc-1]);

      if(currLoc == 0 or (dist > 4.9 and dist < 10)):
        used[next] = True;
        temp.append(shapes[next]);
        perm(shapes, currLoc+1, nShapes, used, temp, permBuffer);
        temp.pop(currLoc);
        used[next] = False;
    
def main():
  image = cv2.imread("map4.png");
  image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY);
  height, width = image.shape;
  directions = [];
  coords = [];
  finalShapes = [];
  used = np.zeros((height, width));
  dX = [-1, 0, 0, 1];
  dY = [0, -1, 1, 0];

  for dx in dX:
    for dy in dY:
      directions.append((dy, dx));

  for y in range(height):
    for x in range(width):
      if(image[y][x] == 0):
          #move to separate thing (kn + n^2 vs kn^3)
          res, shape = countPixels(image, (y, x), directions, used);

          if(res >= 1 and res <= 5):
            for pair in shape:
              coords.append(pair);
          else:
            for tempY, tempX in shape:
              image[tempY][tempX] = 255;
      else:
        image[y][x] = 255;

  nShapes = len(coords);
  t = np.zeros((height, width));

  used = np.zeros(nShapes);
  finalShapes = [];
  perm(coords, 0, nShapes, used, [], finalShapes);
  used = {};
  finalShapes2 = [];

  cv2_imshow(image);

  for shape in finalShapes:
    a = shape[0];
    b = shape[1];
    c = shape[2];
    midY = math.floor((b[0] + c[0]) / 2);
    midX = math.floor((a[1] + b[1]) / 2);

    image = cv2.rectangle(image, (a[1], a[0]), (c[1], c[0]), 0, 1);
    image[midY][midX] = 0;

  cv2_imshow(image);
main();