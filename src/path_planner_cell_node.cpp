#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include<algorithm>
#include<vector>
#include <astar_path_planner/a_star.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>


using namespace std::chrono_literals;
using namespace std;
using namespace ecn;

class path_planner_cell_node : public rclcpp::Node
{
public:
    path_planner_cell_node() : Node("path_planner_cell_node"),start_(0,0),target_(0,0)
    {

        // Log that the node has succesfully started
        RCLCPP_INFO(this->get_logger(), "path planner cell has successfully started");

        // Create a subscrber that get information to update the Map
        map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("mapping", 10,std::bind(&path_planner_cell_node::updateMap, this, std::placeholders::_1));

        // Create subscribers that will receive a geometry_msgs::msgs::Pose message
        //position of the ship
        pos_sub = create_subscription<geometry_msgs::msg::Pose>("position", 10, std::bind(&path_planner_cell_node::positionCallback, this, std::placeholders::_1));
        //position of the target
        targ_pos_sub = create_subscription<geometry_msgs::msg::Pose>("target_position", 10, std::bind(&path_planner_cell_node::targetCallback, this, std::placeholders::_1));

        // Create a timer that will call the timer_callback function
        timer_ = this->create_wall_timer(500ms, std::bind(&path_planner_cell_node::timer_callback, this));

        // Create a publisher on the topic ??? for the PID to get the next waypoint
        waypoint_pub = create_publisher<geometry_msgs::msg::Point>("waypoint", 10);

    };


    void updateMap(const nav_msgs::msg::OccupancyGrid &NewMap)
    {
         int width = NewMap.info.width;
         int height = NewMap.info.height;

         // Resize the map_ vector if necessary
         if (map_.size() != static_cast<size_t>(width) || map_[0].size() != static_cast<size_t>(height))
         {
             map_.resize(width, vector<int>(height, 0));
         }

         int index = 0;
         for (int x = 0; x < width; ++x)
         {

             for (int y = 0; y < height; ++y)
             {
                 map_[x][y] = NewMap.data[index++];
             }
         }
    }

    void positionCallback(const geometry_msgs::msg::Pose &msg)
    {
        start_.x = msg.position.x;
        start_.y = msg.position.y;

        start_.Position::Map_ = map_;
    }

    void targetCallback(const geometry_msgs::msg::Pose &msg)
    {
        target_.x = msg.position.x;
        target_.y = msg.position.y;

        target_.Position::Map_ = map_;
    }

    void timer_callback()
    {
        auto path = Astar(start_,target_);
        next_waypoint.x = path[0].x;
        next_waypoint.y = path[0].y;
        waypoint_pub->publish(next_waypoint);
        RCLCPP_INFO(this->get_logger(), "New target pos: 'x = %f, y = %f'", next_waypoint.x, next_waypoint.y);

    }


    class Point
    {
    public:

        explicit Point(int _x = 0, int _y = 0) : x(_x), y(_y) {}

        ~Point() = default;

        void operator=(const Point &p)
        {
            x = p.x;
            y = p.y;
        }

        // prints the grid with all positions from parent
        virtual void print(const Point &parent) const;


        friend std::ostream& operator<<(std::ostream& out, const Point& p)
        {
            out << "(" << p.x << ", " << p.y << ")";
            return out;
        }

        // 2 positions are equal if they have the same x and y
        bool operator==(const Point &other) const
        {
            return x == other.x && y == other.y;
        }

        double h(const Point &goal, bool use_manhattan) const
        {
            if(use_manhattan)
                return  abs(x-goal.x) + abs(y-goal.y);
            return 1.5 * sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
        }

        int x, y;

        std::vector<std::vector<int>> Map_;
    };

    // a node is a x-y position, we move from 1 each time
    class Position : public Point
    {
        typedef std::unique_ptr<Position> PositionPtr;

    public:
        // constructor from coordinates
        Position(int _x, int _y) : Point(_x, _y) {}

        // constructor from base ecn::Point
        Position(Point p) : Point(p.x, p.y) {}

        int distToParent()
        {
            // in cell-based motion, the distance to the parent is always 1
            return 1;
        }

        std::vector<PositionPtr> children()
        {
            // this method should return  all positions reachable from this one
            std::vector<PositionPtr> generated;

            int x = this->x;
            int y = this->y;

            // Check and add positions above, below, to the left, and to the right
            if (Map_[x][y-1])
                generated.push_back(std::make_unique<Position>(x, y - 1));

            if (Map_[x][y+1])
                 generated.push_back(std::make_unique<Position>(x, y + 1));

            if (Map_[x-1][y])
                generated.push_back(std::make_unique<Position>(x - 1, y));

            if (Map_[x+1][y])
                generated.push_back(std::make_unique<Position>(x + 1, y));

            return generated;
        }
    };


    //Talk and subscribe
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pos_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr targ_pos_sub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub;
    rclcpp::TimerBase::SharedPtr timer_;


    using Map = vector<vector<int>>;

    //Variables
    Position start_;
    Position target_;
    Map map_;

    geometry_msgs::msg::Point next_waypoint;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_planner_cell_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
