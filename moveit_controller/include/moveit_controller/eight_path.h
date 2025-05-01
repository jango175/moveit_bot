#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

struct Point2D {
  double x;
  double y;
};

std::vector<Point2D> generate_eight_shaped_path(
  double start_x, 
  double start_y, 
  double path_length,
  int num_points = 100)
{
  std::vector<Point2D> path;

  // Calculate the radius of each circle in the figure-8
  // The total length of a figure-8 made of two circles is approximately 4*r
  double radius = path_length / 4.0;
  Point2D point;

  // Generate points along the figure-8 path
  for (int i = 0; i < num_points; ++i) {
    double t = 2.0 * M_PI * i / num_points;

    // Parametric equation for a figure-8
    // Using lemniscate of Gerono: x = cos(t), y = sin(t)*cos(t)
    // Scaled and translated to our desired position and size
    point.x = start_x + radius * sin(t);
    point.y = start_y + radius * sin(t) * cos(t);

    path.push_back(point);
  }

  // Back to the starting point
  point.x = start_x;
  point.y = start_y;
  path.push_back(point);

  return path;
}
