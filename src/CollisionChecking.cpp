///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Author: Ryan Luna
//////////////////////////////////////

#include "CollisionChecking.h"

// Definition of a line segment between (x1, y1) and (x2, y2)
struct Line
{
    double x1, y1;
    double x2, y2;
};

// Check the segments l1 and l2 for an intersection.  If these two segments intersect
// at a single point, return true.
bool lineLineIntersection(const Line& l1, const Line& l2)
{
    // re-encode the lines as rays
    // direction vector for l1
    std::vector<double> d1(2);
    d1[0] = l1.x2 - l1.x1;
    d1[1] = l1.y2 - l1.y1;

    // direction vector for l2
    std::vector<double> d2(2);
    d2[0] = l2.x2 - l2.x1;
    d2[1] = l2.y2 - l2.y1;

    // Compute determinant of direction vectors
    double det = d1[1] * d2[0] - d1[0] * d2[1];
    if (fabs(det) < 1e-6) // rays are parallel.
        return false;

    double dx, dy;
    // Do not divide by zero when l2 is horizontal - just swap l1 and l2
    if (fabs(d2[1]) < 1e-4)
    {
        std::swap(d1[0], d2[0]);
        std::swap(d1[1], d2[1]);

        dx = l2.x1 - l1.x1;
        dy = l2.y1 - l1.y1;
        det = -det;
    }
    else
    {
        dx = l1.x1 - l2.x1;
        dy = l1.y1 - l2.y1;
    }

    // time parameter for the intersection of ray 1
    double t1 = (dx * d2[1] - dy * d2[0]) / det;
    if (fabs(t1) < 1e-6)
        t1 = 0.0;

    // time parameter for the intersection of ray 2
    double t2 = (dy + t1 * d1[1]) / d2[1];
    if (fabs(t2) < 1e-6)
        t2 = 0.0;

    if (t1 < 0 || t2 < 0)   // Both scalars must be non-negative for an intersection
        return false;

    if (t1 > 1 || t2 > 1)   // Both scalars must be <= 1 for an intersection
        return false;

    return true;            // Yep, lines intersect
}

// Intersect the point (x,y) with the set of rectangles.  If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
        if (x >= obstacles[i].x && x <= obstacles[i].x + obstacles[i].width)
            if (y >= obstacles[i].y && y <= obstacles[i].y + obstacles[i].height)
                return false;
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles.  If the circle
// lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{
    // Check whether the center of the circle is inside the Minkowski obstacle
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
        // Inflate x-axis by radius
        if (x >= (obstacles[i].x - radius) && x <=(obstacles[i].x + obstacles[i].width + radius))
            if (y >= obstacles[i].y  && y <= obstacles[i].y + obstacles[i].height)
                return false;

        // Inflate y-axis by radius
        if (x >= obstacles[i].x && x <= obstacles[i].x + obstacles[i].width)
            if (y >= (obstacles[i].y - radius)  && y <= (obstacles[i].y + obstacles[i].height + radius))
                return false;

        // Check the corners - C-obstacle has rounded edges...
        // lower-left
        double dx = obstacles[i].x - x;
        double dy = obstacles[i].y - y;
        if (sqrt(dx * dx + dy * dy) <= radius)
            return false;

        // upper-left
        dx = obstacles[i].x - x;
        dy = obstacles[i].y + obstacles[i].height - y;
        if (sqrt(dx * dx + dy * dy) <= radius)
            return false;

        // upper-right
        dx = obstacles[i].x + obstacles[i].width - x;
        dy = obstacles[i].y + obstacles[i].height - y;
        if (sqrt(dx * dx + dy * dy) <= radius)
            return false;

        // lower-right
        dx = obstacles[i].x + obstacles[i].width - x;
        dy = obstacles[i].y - y;
        if (sqrt(dx * dx + dy * dy) <= radius)
            return false;
    }

    // no collisions
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles.
// If the square lies outside of all obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
    double halfLength = sideLength / 2.0;

    // Coordinates of the square geometry in local coordinate frame with origin at center of square
    std::vector<std::pair<double, double> > pts;
    pts.push_back(std::make_pair(-halfLength, -halfLength));
    pts.push_back(std::make_pair(-halfLength,  halfLength));
    pts.push_back(std::make_pair( halfLength,  halfLength));
    pts.push_back(std::make_pair( halfLength, -halfLength));

    // Rigid transformation of the square to the given configuration
    for(size_t i = 0; i < pts.size(); ++i)
    {
        double newX = pts[i].first * cos(theta) - pts[i].second * sin(theta) + x;
        double newY = pts[i].first * sin(theta) + pts[i].second * cos(theta) + y;
        pts[i] = std::make_pair(newX, newY);
    }

    // Collision checking by intersecting all-pairs of line segments between robot and environment
    std::pair<double, double> prev = pts.back();
    for(size_t i = 0; i < pts.size(); ++i)  // Enumerating line segments of the robot
    {
        Line roboLine;
        roboLine.x1 = prev.first;
        roboLine.y1 = prev.second;
        roboLine.x2 = pts[i].first;
        roboLine.y2 = pts[i].second;
        prev = pts[i];

        for(size_t j = 0; j < obstacles.size(); ++j) // Enumerating line segments of the obstacles
        {
            for(int k = 0; k < 4; ++k)  // each obstacle has four sides
            {
                Line obstacleLine;
                obstacleLine.x1 = obstacleLine.x2 = obstacles[j].x;
                obstacleLine.y1 = obstacleLine.y2 = obstacles[j].y;

                switch (k)
                {
                    case 0: // left
                        obstacleLine.y2 += obstacles[j].height;
                        break;
                    case 1: // top
                        obstacleLine.y1 += obstacles[j].height;
                        obstacleLine.x2 += obstacles[j].width;
                        obstacleLine.y2 += obstacles[j].height;
                        break;
                    case 2: // right
                        obstacleLine.x1 += obstacles[j].width;
                        obstacleLine.y1 += obstacles[j].height;
                        obstacleLine.x2 += obstacles[j].width;
                        break;
                    case 3: // bottom
                        obstacleLine.x1 += obstacles[j].width;
                        break;
                }

                // collision check this robot and obstacle line segment pair
                if (lineLineIntersection(roboLine, obstacleLine))
                    return false;
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// SUPER SPECIAL CASES ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Note: I would not expect people to casually handle these cases, since the environment is composed of line segments
    // instead of solid shapes.  Nevertheless, somebody will ask about this.  Polygon intersection is difficult, even for
    // simple convex shapes.  The approaches below use the assumptions wisely to avoid the issue of polygon intersection.

    // The robot is completely inside an obstacle - not covered by line intersections
    // If this happens, at least one vertex of the robot is inside an obstacle.  Easy to check because axis-alignment
    for(size_t i = 0; i < pts.size(); ++i)
        if (!isValidPoint(pts[i].first, pts[i].second, obstacles))
            return false;

    // The obstacle is completely inside the robot - also not covered by line intersections
    // We can do the same as when the robot is inside the obstacle, but this gets
    // tricky since the robot is not axis aligned.  One workaround is to cheat and
    // put the obstacle coordinates in the local robot frame (which is axis-aligned).
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
        for(int j = 0; j < 4; ++j)
        {
            // Extract the jth coordinate of this obstacle
            double px = obstacles[i].x;
            double py = obstacles[i].y;
            switch (j)
            {
                case 1: // top left
                    py += obstacles[i].height;
                    break;
                case 2: // top right
                    px += obstacles[i].width;
                    py += obstacles[i].height;
                    break;
                case 3: // bottom right
                    px += obstacles[i].width;
                    break;
            }

            // Invert the robot frame at (x,y,theta) and multiply with coordinate vector (px,py) to
            // put the obstacle coordinates in the local robot frame - 1337H4X
            double plx =  px * cos(theta) + py * sin(theta) - x * cos(theta) - y * sin(theta);
            double ply = -px * sin(theta) + py * cos(theta) - y * cos(theta) + x * sin(theta);

            // Now check whether (plx, ply) intersects with the robot in its local frame.
            // Simple since the local frame is axis aligned
            if (plx >= -halfLength && plx <= halfLength)
                if (ply >= -halfLength && ply <= halfLength)
                    return false;
        }
    }
    return true;
}
