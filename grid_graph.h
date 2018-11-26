#include <cmath>

class GridGraphEdge;

struct NodeConnections {
private:
    int degree;
public:
    GridGraphEdge * connections_[4];
    
    NodeConnections(): degree(0) {}

    int degree() const {
        return degree;
    }
    
    void resetConnections() {
        for (int i = 0; i < 4; i++) {
            connections_[i] = nullptr;
        }   
        degree = 0;
    }

    void connect(GridGraphEdge * new_one) {
        degree++;
        for (int i = 0; i < 4; i++) {
            if (!connections_[i]) {
                connections_[i] = new_one;
                return;
            }
        }
        assert(0 && "More than 4 connections are not allowed");
    }
};


class GridGraphNode {
    static unsigned row_len_; 
    friend SteinerGridGraph::x_ensure_size; //which sets row_len

    unsigned x_;
    unsigned y_;
public:
    GridGraphNode(unsigned x, unsigned y):
        x_(x), y_(y) {}
    GridGraphNode(GridGraphNode &) = delete;
    GridGraphNode(GridGraphNode &&) = delete;

    double dist(const GridGraphNode & other) const { 
        unsigned xdist = other.x_ - x_;
        unsigned ydist = other.y_ - y_;
        return std::sqrt(xdist*xdist + ydist*ydist);
    }

    static size_t hash(const GridGraphNode & in) {
        return x_*row_len_ + y;
    }
    
    bool operator == (const GridGraphNode & other) const {
        return (x_ == other.x_) && (y_ == other.y_);
    }
};

template<class T> class InnerHasher {
public:
    size_t operator()(const T &node) const {
        return node->hash();
    }
};

struct GridGraphEdge {
    const GridGraphNode & first_;
    const GridGraphNode & second_;
    const double length_;

    GridGraphEdge(const GridGraphNode & one,
                  const GridGraphNode & other):
        first_(one), second_(other), length_(one.dist(other_)) {}
    GridGraphEdge(GridGraphEdge &) = delete;
    GridGraphEdge(GridGraphEdge &&) = delete;
};


