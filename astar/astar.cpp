#include <planning/astar.h>
#include <planning/obstacle_distance_grid.hpp>
#include <algorithm>
#include <cmath>
#include <common/point.hpp>

using namespace std;

gridPoint poseToGridPoint(pose_xyt_t& pose, const ObstacleDistanceGrid& distances);

pose_xyt_t gridPointToPose(gridPoint point, const ObstacleDistanceGrid& distances);

class AStar
{
public:

    float cell_dist;
    float diagonal_cell_dist;

private:
    std::vector<gridPoint> open_set;
    std::vector<gridPoint> closed_set;

    //function to verify if a line between two poses is valid
    bool testLine(int first, int last, const ObstacleDistanceGrid& distances, const SearchParams& params)
	{
		return true;
	}

	robot_path_t generatePath(std::vector<int>& point_path, pose_xyt_t& start, pose_xyt_t& goal, const ObstacleDistanceGrid& distances)
	{
		//initialize return path
		robot_path_t path;

		//add start
		path.path.push_back(start);

		pose_xyt_t next_pose;

		//generate poses positions from points (must go backwards)
		for (int i = point_path.size() - 1; i > -1; --i)
		{
			//convert to real world system
			next_pose.x = ((float)closed_set[point_path[i]].x * distances.metersPerCell()) + distances.originInGlobalFrame().x;
			next_pose.y = ((float)closed_set[point_path[i]].y * distances.metersPerCell()) + distances.originInGlobalFrame().y;
			//set time to 0
			next_pose.utime = 0;

			path.path.push_back(next_pose);
		}

		//add goal snd set size
		path.path.push_back(goal);
		path.path_length = path.path.size();

		//generate angles for each pose (not including start and goal)
		float dx, dy;
		for (unsigned int i = 1; i < path.path.size() - 1; ++i)
		{
			//set angle to halfway between 
			dx = (float)(path.path[i+1].x - path.path[i-1].x);
			dy = (float)(path.path[i+1].y - path.path[i-1].y);

			//make sure x isn't 0
			if (dx == 0.0f)
			{
				dx = 0.000001f;
			}

			path.path[i].theta = tan(dy/dx);
		}

    	return path;
	}


public:

    //open set functions
    gridPoint openSetPop()
    {
        gridPoint shadow_point = open_set.front();
        pop_heap(open_set.begin(), open_set.end());
        open_set.pop_back();
        return shadow_point;
    }

    void openSetPush(gridPoint& point)
    {
        open_set.push_back(point);
        push_heap(open_set.begin(), open_set.end());
    }

    bool openSetEmpty()
    {
        return open_set.empty();
    }

    int openSetFind(gridPoint point)
    {
        auto it = find(open_set.begin(), open_set.end(), point);

        if (it == open_set.end())
        {
            return -1;
        }
        return (it - open_set.begin());
    }

    void openSetReplace(int index, gridPoint point)
    {
        open_set[index] = point;
        make_heap(open_set.begin(), open_set.end());
    }

    float openSetGScore(int index)
    {
        return open_set[index].g;
    }

    //closed set functions
    void closedSetPush(gridPoint& point)
    {
        closed_set.push_back(point);
    }

    bool inClosedSet(gridPoint& point)
    {
        auto it = find(closed_set.begin(), closed_set.end(), point);

        if (it == closed_set.end()) {
            return false;
        }

        return true;
    }

    int parent()
    {
        //return index of most recent point added (current)
        return closed_set.size() - 1;
    }

    robot_path_t buildPath(pose_xyt_t start, pose_xyt_t goal, const ObstacleDistanceGrid& distances, 
    	const SearchParams& params, AStar& search, gridPoint start_grid)
	{
        robot_path_t tmp_path;
        tmp_path.utime = start.utime;
        gridPoint prev_grid = closed_set[0];
        for (size_t i = closed_set.size() - 1; true;) 
        {
            pose_xyt_t curr = gridPointToPose(closed_set[i], distances);
            curr.utime = 0;
            i = closed_set[i].parent;
            tmp_path.path.push_back(curr);
            
            if (closed_set[i].x == start_grid.x && closed_set[i].y == start_grid.y)
            {
                break;
            }
            
        }
        std::reverse(tmp_path.path.begin(), tmp_path.path.end());
        tmp_path.path_length = tmp_path.path.size();
        for (int i = 0; i < tmp_path.path_length ; i++)
        {
            if (i == 0)
                tmp_path.path[i].theta = start.theta;
            else
            {
                Point<double> start = {tmp_path.path[i-1].x, tmp_path.path[i-1].y};
                Point<double> end = {tmp_path.path[i].x, tmp_path.path[i].y};
                tmp_path.path[i].theta = angle_to_point(start, end);
            }
        }
        
        
        return tmp_path;

	    //store path as a series of indexes to closed set. That way angles for poses can be established later
	    std::vector<int> point_path;

	    int last_point = closed_set.size()-1;
	    int test_point = last_point;

	    //add last point to path because algorithm works with parents and therefore in reverse
	    point_path.push_back(last_point);

		//Check corner case of if first point equals last point
		if (last_point == 0)
		{
			robot_path_t path;

			//just return both poses
			path.utime = start.utime;
		    path.path.push_back(start);    
		    path.path.push_back(goal);    
		    path.path_length = path.path.size();

		    return path;
		}

		//loop indefinitely
		while(1)
		{
			//try to make pose jumps as long as possible
    		while ((*this).testLine(closed_set[test_point].parent , last_point, distances, params))
    		{
    			//increment test_point to its parent
    			test_point = closed_set[test_point].parent;

    			//check if start point is reached
    			if (test_point == 0)
    			{
    				//add start to end of path
    				point_path.push_back(test_point);
    			}
    		}

    		//test point is moved as far as possible so add it to path
    		point_path.push_back(test_point);

    		//set last point to test point and start again
    		last_point = test_point;
		}
	}

    float calculateHScore(gridPoint& current, gridPoint& goal)
    {
        int x_dist = abs(goal.x - current.x);
        int y_dist = abs(goal.y - current.y);
        int straight_dist = abs(x_dist - y_dist);
        int diagonal_dist = (x_dist + y_dist - straight_dist)/2;

        return ((float) straight_dist)*cell_dist + 
            ((float)diagonal_dist)*diagonal_cell_dist;
    }

};



void checkNeighbor(int dx, int dy, gridPoint& current, gridPoint& goal_point, const ObstacleDistanceGrid& distances, 
    const SearchParams& params, AStar& search)
{
    //initialize neighbor
    gridPoint neighbor;
    neighbor.x = current.x + dx;
    neighbor.y = current.y + dy;

    //check if neighbor is valid cell
    if (!distances.isCellInGrid(neighbor.x, neighbor.y)) {
        //cout << "checkNeighbour end" << endl;
        return;
    }

    //check if neighbor is too close to obstacle
    if (distances(neighbor.x, neighbor.y) <= params.minDistanceToObstacle) {
        //cout << "checkNeighbour end" << endl;
        return;
    }

    //check if neighbor is already in closed set
    if (search.inClosedSet(neighbor)) {
        //cout << "checkNeighbour end" << endl;
        return;
    }

    //calculate h score
    neighbor.h = search.calculateHScore(neighbor, goal_point);

    float cellDist = abs(dx)^abs(dy) ? search.cell_dist : search.diagonal_cell_dist;

    //calculate possible g score
    if (distances(neighbor.x, neighbor.y) < params.maxDistanceWithCost)
    {
    	neighbor.g = current.g + cellDist;
    		//+ pow(params.maxDistanceWithCost - distances(neighbor.x, neighbor.y), params.distanceCostExponent);
    }
    else
    {
    	neighbor.g = current.g + cellDist;
    }

    //look for neighbor in open set
    int index = search.openSetFind(neighbor);

    // if neighbor is present in open set
    if (index != -1)
    {
        //compare g scores
        if (search.openSetGScore(index) > neighbor.g)
        {
            //set neighbor parent
            neighbor.parent = search.parent();

            //update g value
            neighbor.f = neighbor.g + neighbor.h;

            //set F score based on last f score
            search.openSetReplace(index, neighbor);
        }
        return;
    }
    else 
    {
        //set neighbor parent
        neighbor.parent = search.parent();
    }

    //set f score (This function is still incomplete)
    neighbor.f = neighbor.g + neighbor.h;

    //put neighbor into open set
    search.openSetPush(neighbor);
}

gridPoint poseToGridPoint(pose_xyt_t& pose, const ObstacleDistanceGrid& distances)
{
    gridPoint point;

    point.x = int((pose.x - distances.originInGlobalFrame().x)*distances.cellsPerMeter());
    point.y = int((pose.y - distances.originInGlobalFrame().y)*distances.cellsPerMeter());

    point.g = 0.0f;
    point.h = 0.0f;

    return point;
};

pose_xyt_t gridPointToPose(gridPoint point, const ObstacleDistanceGrid& distances)
{
    pose_xyt_t pose;

    pose.x = (double) point.x*distances.metersPerCell() + distances.originInGlobalFrame().x;
    pose.y = (double) point.y*distances.metersPerCell() + distances.originInGlobalFrame().y;

    return pose;
}

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{

	//convert start and goal into gridPoints
    gridPoint start_point = poseToGridPoint(start, distances);
    gridPoint goal_point = poseToGridPoint(goal, distances);

    //Initialize search 
    AStar search;

    search.cell_dist = distances.metersPerCell();
    search.diagonal_cell_dist = distances.metersPerCell() * sqrt(2);

    //add start to open set
    search.openSetPush(start_point);

    //holder
    gridPoint current;

    cout << goal_point.x << " " << goal_point.y << endl;

    //start algorithm
    while (!search.openSetEmpty())
    {
        //pop open set top
        current = search.openSetPop();

        //add current to closed set
        search.closedSetPush(current);

        //check if goal is reached
        if (current == goal_point)
        {
            std::cout << "A* found path" << std::endl;
            // path was found, build it and return it          
            return search.buildPath(start, goal, distances, params, search, start_point);
        }
        //try 4 adjacent neighbors
        checkNeighbor(1, 0, current, goal_point, distances, params, search);
        checkNeighbor(0, 1, current, goal_point, distances, params, search);
        checkNeighbor(-1, 0, current, goal_point, distances, params, search);
        checkNeighbor(0, -1, current, goal_point, distances, params, search);
        checkNeighbor(1, 1, current, goal_point, distances, params, search);
        checkNeighbor(-1, -1, current, goal_point, distances, params, search);
        checkNeighbor(-1, 1, current, goal_point, distances, params, search);
        checkNeighbor(1, -1, current, goal_point, distances, params, search);
    }
    //no path was found
    std::cout << "A* No Path Found" << std::endl; 
    
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}
