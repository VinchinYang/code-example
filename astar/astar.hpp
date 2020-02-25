#ifndef PLANNING_ASTAR_H_
#define PLANNING_ASTAR_H_

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>

class ObstacleDistanceGrid;

/// @breif Search Parameters
struct SearchParams
{
    /// @brief The minimum distance a robot can be from an obstacle before
    ///        a collision occurs
    double minDistanceToObstacle;   
                                    

    /// @brief The maximum distance from an obstacle that has an associated cost.          
    double maxDistanceWithCost;     
                                    
    /// @brief exponent to apply to the distance cost
    double distanceCostExponent;    
                                    
};

struct gridPoint
{
    int x;
    int y;
    float g;
    float h;
    float f;
    /// @brief corresponds to index in closed set
    int parent;

    bool operator==(const gridPoint& rhs)
    {
        if (x == rhs.x && y == rhs.y)
            return true;
        else
            return false;
    }

    bool operator <(const gridPoint& rhs)
    {
        if (f == rhs.f)
            return h > rhs.h;
        else
            return f > rhs.f;
    }
};

/// @brief search_for_path uses an A* search to find a path from the start to 
///        goal poses
///
/// @param start           Starting pose of the robot
/// @param goal            Desired goal pose of the robot
/// @param distances       Distance to the nearest obstacle for each cell in the 
///                        grid.
/// @param params          Parameters specifying the behavior of the A* search.
/// @return The path found to the goal, if one exists. If the goal is 
///         unreachable, then a path with just the initial
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

#endif // PLANNING_ASTAR_H_
