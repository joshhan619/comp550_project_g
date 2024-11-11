#include <ompl/base/spaces/SE2StateSpace.h>

// Axis aligned bounding box
struct AABB
{
    double minX, minY;
    double maxX, maxY;

    bool pointInsideAABB(double x, double y, double rad) const;
    bool pointInsideAABB(double x, double y) const;
};

// Rectangle
struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

#define MAKE_POINT(x,y) std::make_pair(x, y)
typedef std::pair<double, double> Point2D;

// Tests if two lines intersect
bool lineLineIntersection(const Point2D& p1, const Point2D& p2, const Point2D& q1, const Point2D& q2);

// Transforms from rectangles to axis-aligned bounding box representation
AABB rectangleToAABB(const Rectangle &obstacle);

// Tests if state is valid (for a point robot)
bool isValidStatePoint(const ompl::base::State* state, const std::vector<Rectangle>& obstacles);
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles);
// Tests if state is valid (for a square robot)
bool isValidStateSquare(const ompl::base::State* state, double sideLen, const std::vector<Rectangle>& obstacles);
