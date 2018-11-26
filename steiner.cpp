#include <iostream>
#include <functional>
#include <algorithm>
#include <unordered_set>

struct SteinerGridGraph {
private:
    unsigned xmax_, ymax_;

    static unsigned x_ensure_size(unsigned x, unsigned y) {
        assert(1.*x*y < 1 + std::numeric_limits<size_t>::max());
        GridGraphNode::row_len_ = y;
        return x;
    }

public:
    std::vector<GridGraphNode> core_pts_;
    std::unordered_set<GridGraphNode, InnerHasher> steiner_pts_;
 
template <typename NodeInput>
    SteinerGridGraph (NodeInput beg, NodeInput end, unsigned x_size, unsigned y_size):
        xmax_(x_ensure_size(x_size, y_size)), ymax_(y_size) {
        std::copy(beg, end, std::back_inserter(core_pts_));
    }

    unsigned computeMST() { //total length is returned

    }

    unsigned updateMST(const GridGraphNode & new_one) {

    }

    void ensure_degree() {
        for (auto cur = steiner_pts_.begin(); cur != steiner_pts_.end(); ++cur) {
            
        }
    }
};

class SteinerAlgo {
    const SteinerGridGraph & graph_;
    std::unordered_set<GridGraphNode, decltype(&GridGraphNode::hash)> candidate_pts_;

    void extend_candidates(core_pts_::iter_type p1, core_pts::iter_type p2) {
        std::pair<GridGraphNode, GridGraphNode> st_pts = p1->steinerPoints(*p2);
        auto [iter, done] = candidate_pts_.emplace(st_pts.first, st_pts.second);
        assert (done && "Can not add candidates");
    }

    void build_candidates() {
        auto p_end = graph_.core_pts_.end();

        for (auto p_handle = graph_.core_pts_.begin();
            p_handle != p_end;
            ++p_handle) {
            for (auto cur = graph_.core_pts_.begin(); cur != p_handle; ++cur) {
                extend_candidates(p_handle, cur);
            }
        }  
        //may be excessive with respect to amount of points included
        //TODO speed up by excluding points on outer border
    }

    void choose_candidate (candidate_pts_.iter_type chosen) {
        
    }


public:   
    SteinerAlgo(const SteinerGridGraph & gr):
        graph_(gr) {}

    void computeSteinerMST() {
        unsigned cur_length = graph_.computeMST();
        build_candidates();

        auto cand_end = candidate_pts_.end();
        
        //TODO if candidates were sorted by a heuristic, it would have improved perf
        bool progress = true;
        while (progress) {
            progress = false;
            for (auto cur = candidate_pts_.begin();
                cur != cand_end; ++cur) {
                unsigned new_length = graph_.updateMST(*cur);
                if (new_length < cur_length) { //included new point
                    choose_candidate(cur);
                    graph_.ensure_degree();
                    //TODO think if we want to write deleted pts back to candidates
                    progress = true;
                    cur_length = new_length;
                    break;
                }
            }
        }
    }
};
