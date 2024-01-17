#ifndef POINT_H
#define POINT_H

#include <vector>
#include <memory>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <nav_msgs/msg/occupancy_grid.hpp>

class Point {
public :

    Point(int x=0, int y=0) : x_(x), y_(y) {}

    typedef std::unique_ptr<Point> Ptr;
    typedef std::vector<Ptr> Vector;


    bool is(const Point &goal) const
    {
        return x_ == goal.x_ && y_ == goal.y_;
    }



    Vector children()
    {
        Vector children;

        // Assuming you have some way to check if a point is valid in your grid
        for(int dx = -1; dx <= 1; ++dx)
        {
            for(int dy = -1; dy <= 1; ++dy)
            {
                if(dx == 0 && dy == 0) continue; // Skip the current point
                Point child;
                child.x_ = x_ + dx;
                child.y_ = y_ + dy;
                if(isValid(child)) { // isValid should check if the point is within grid bounds and not an obstacle
                    children.push_back(std::make_unique<Point>(child));
                }
            }
        }
        return children;
    }

    bool isValid(const Point& candidate)
    {
        //We get the informations of the map
        auto width = map_.info.width;
        auto height = map_.info.height;
        auto occupancy_grid = map_.data;
        //storing of the frame origin
        auto Ox = map_.info.origin.position.x;
        auto Oy = map_.info.origin.position.y;

        bool is_Valid = true;

        //we check if the candidate is out of the map
        if(not(candidate.x_>Ox-width/2 && candidate.x_<Ox+width/2))
        {
            is_Valid = false;
        }
        if(not(candidate.y_>Oy-height/2 && candidate.y_<Oy+height/2))
        {
            is_Valid = false;
        }

        //we check if the candidate is occuped
        if(occupancy_grid[candidate.y_*width+candidate.x_]>0.5)
        {
            is_Valid = false;
        }

        return is_Valid;
    }

    int h(const Point &goal) {
        // Manhattan distance
        return std::abs(x_ - goal.x_) + std::abs(y_ - goal.y_);

        // Or Euclidean distance
        // return std::sqrt((x - goal.x) * (x - goal.x) + (y - goal.y) * (y - goal.y));
    }


    int distToParent(){return 1;}

    //point variable declaration
    int x_,y_;
    nav_msgs::msg::OccupancyGrid map_;

};

#endif // POINT_H
