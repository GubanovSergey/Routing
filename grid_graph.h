#include <cmath>
#include <utility>
#include <cassert>

#include "tinyxml2.h"

struct GridGraphNode {
    unsigned x_;
    unsigned y_;

    GridGraphNode(unsigned x, unsigned y):
        x_(x), y_(y) {}

    double dist(const GridGraphNode & other) const { 
        unsigned xdist = std::abs((int)other.x_ - (int)x_);
        unsigned ydist = std::abs((int)other.y_ - (int)y_);
        return xdist + ydist;
    }

    bool operator == (const GridGraphNode & other) const {
        return (x_ == other.x_) && (y_ == other.y_);
    }

    std::pair<GridGraphNode, GridGraphNode> steinerPoints(const GridGraphNode & other) {
        return std::make_pair(GridGraphNode(x_, other.y_), 
                              GridGraphNode(other.x_, y_));
    }

    std::pair<GridGraphNode, GridGraphNode> horizontalSegment(const GridGraphNode & dest) {
        return std::make_pair(GridGraphNode(x_, y_), GridGraphNode(dest.x_, y_));
    }
    
    std::pair<GridGraphNode, GridGraphNode> verticalSegment(const GridGraphNode & dest) {
        return std::make_pair(GridGraphNode(x_, y_), GridGraphNode(x_, dest.y_));
    }

    void dump_via(tinyxml2::XMLElement * netElement, bool between_ms = false) {
        using namespace tinyxml2;
    
        XMLElement * elem = netElement->GetDocument()->NewElement("point");
        elem->SetAttribute("x", x_); 
        elem->SetAttribute("y", y_); 
        const char * l = between_ms ? "m2_m3" : "pins_m2";
        elem->SetAttribute("layer", l); 
        elem->SetAttribute("type", "via");
        netElement->InsertEndChild(elem);
    }
};

struct SteinerGridGraph {
    std::vector<GridGraphNode> pts_;
    unsigned core_pts_num_;
    boost::dynamic_bitset<> active_;
 
template <typename NodeInput>
    SteinerGridGraph (NodeInput beg, NodeInput end) {
        auto inserter = std::back_inserter(pts_);
        unsigned cnt = 0;
        for (auto cur = beg; cur != end; ++cur) {
            *(inserter) = *cur;
            cnt++; ++inserter;
        }        
        core_pts_num_ = cnt;
    }

    unsigned pt_next_active (unsigned num) {
        if (num + 1 < core_pts_num_) {
            return num + 1;
        }
        decltype(active_)::size_type next;
        if (num + 1 == core_pts_num_)
            next = active_.find_first();
        else
            next = active_.find_next(num - core_pts_num_);

        if (next == active_.npos) {
            return pts_.size();
        }
        else {
            return next + core_pts_num_;
        }
    }

    unsigned cand_pts_num() {
        return pts_.size() - core_pts_num_;
    }
};

struct NodeConnections {
private:
    int degree_;
public:
    unsigned connections_[4]; //edge id
    
    NodeConnections(): degree_(0) { //0 - special value
        resetConnections();
    }

    int degree() const {
        return degree_;
    }
    
    void resetConnections() {
        for (int i = 0; i < 4; i++) {
            connections_[i] = 0;
        }   
        degree_ = 0;
    }

    void connect(unsigned new_one) {
        degree_++;
        for (int i = 0; i < 4; i++) {
            if (!connections_[i]) {
                connections_[i] = new_one + 1;
                return;
            }
        }
        assert(0 && "More than 4 connections are not allowed");
    }

    bool is_connected(unsigned index) {
        return connections_[index] != 0;
    }

    unsigned connection(unsigned index) { //to edge id if not zero
        assert(connections_[index] != 0);
        return connections_[index] - 1;
    }
};

struct GridGraphEdge {
    unsigned first_;
    unsigned second_;
    double length_;

    GridGraphEdge(SteinerGridGraph & graph,
                  unsigned one, unsigned other):
        first_(one), 
        second_(other), 
        length_(graph.pts_[one].dist(graph.pts_[other])) 
    {}

    bool operator >(const GridGraphEdge & other) const {
        return length_ > other.length_;
    }

    unsigned other(unsigned node_id) const {
        return (node_id == first_) ? first_ : second_;
    }
};

