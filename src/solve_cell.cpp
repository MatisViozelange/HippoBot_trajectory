#include<a_star.h>
#include<vector>

using namespace std;
using namespace ecn;

using Map = vector<vector<int>>;
int obstacle;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:
    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

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
        if (Position::Map_[x][y-1]!=obstacle)
            generated.push_back(std::make_unique<Position>(x, y - 1));

        if (Position::Map_[x][y+1]!=obstacle)
             generated.push_back(std::make_unique<Position>(x, y + 1));

        if (Position::Map_[x-1][y]!=obstacle)
            generated.push_back(std::make_unique<Position>(x - 1, y));

        if (Position::Map_[x+1][y]!=obstacle)
            generated.push_back(std::make_unique<Position>(x + 1, y));

        return generated;
    }
};

void pathPlan(Position _start, Position _goal)
{
    ecn::Astar(_start,_goal);
}
