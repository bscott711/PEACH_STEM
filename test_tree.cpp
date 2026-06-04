#include <stdio.h>
#include <math.h>
#include <stdlib.h>

float hash_fn(int a, int b) {
  float h = sinf(a * 12.9898f + b * 78.233f) * 43758.5453123f;
  return h - floorf(h);
}

int leafCount = 0;

void drawBranch(float x, float y, float len, float angle, int depth, int pathIndex) {
  const int maxDepth = 4;
  
  float endX = x + len * sinf(angle);
  float endY = y - len * cosf(angle);

  if (depth < maxDepth) {
    float leftLenMod = 0.65f + hash_fn(pathIndex, 1) * 0.15f;
    float rightLenMod = 0.65f + hash_fn(pathIndex, 2) * 0.15f;
    float leftAngleMod = 0.45f + hash_fn(pathIndex, 3) * 0.4f;
    float rightAngleMod = 0.45f + hash_fn(pathIndex, 4) * 0.4f;

    drawBranch(endX, endY, len * leftLenMod, angle - leftAngleMod, depth + 1, pathIndex * 2);
    drawBranch(endX, endY, len * rightLenMod, angle + rightAngleMod, depth + 1, pathIndex * 2 + 1);

    if (hash_fn(pathIndex, 5) > 0.5f) { 
       drawBranch(endX, endY, len * 0.5f, angle + (hash_fn(pathIndex, 6) * 0.2f - 0.1f), depth + 1, pathIndex * 2 + 2);
    }
  }

  if (depth == maxDepth) {
    leafCount++;
    printf("Leaf %d: pathIndex=%d, x=%.1f, y=%.1f\n", leafCount, pathIndex, endX, endY);
  }
}

int main() {
    drawBranch(64.0f, 64.0f, 12.0f, 0.0f, 0, 1);
    printf("Total leaves: %d\n", leafCount);
    return 0;
}
