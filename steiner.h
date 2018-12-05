#include <iostream>
#include <functional>
#include <algorithm>
#include <vector>
#include <iterator>
#include <queue>
#include <map>
#include <boost/dynamic_bitset.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/disjoint_sets.hpp>

#include "grid_graph.h"
#include "tinyxml2.h"

struct SteinerMST {
public:
    SteinerGridGraph & graph_;
    std::vector<GridGraphEdge> edges_;
    std::vector<NodeConnections> cnts_;
    unsigned length_;

    template <typename PtsCont>
    void dump_segment(tinyxml2::XMLElement * netElement, 
            PtsCont && pts, bool via1 = false, bool via2 = false) {
        using namespace tinyxml2;
        auto& [p1, p2] = pts;
        bool horiz = (p1.y_ == p2.y_);
        assert (horiz || p1.x_ == p2.x_); //strait
        
        XMLElement * elem = netElement->GetDocument()->NewElement("segment");
        elem->SetAttribute("x1", p1.x_); 
        elem->SetAttribute("y1", p1.y_); 
        elem->SetAttribute("x2", p2.x_); 
        elem->SetAttribute("y2", p2.y_); 
            
        const char * l = horiz ? "m2" : "m3";
        elem->SetAttribute("layer", l); 
        netElement->InsertEndChild(elem); 
        
        if (!horiz) {
            if (via1)
                p1.dump_via(netElement, true);
            if (via2)
                p2.dump_via(netElement, true);
        } else {
            assert((via1 == via2) && (via2 == false));
        }
    }

    void dump_edge(tinyxml2::XMLElement * netElement, GridGraphEdge & edge) {
        using namespace tinyxml2;
        assert(cnts_.size() == graph_.pts_.size());
        static boost::dynamic_bitset<> viad(cnts_.size());
        
        auto & p1 = graph_.pts_[edge.first_];
        auto & p2 = graph_.pts_[edge.second_];
       
        bool hor = p1.y_ == p2.y_;
        bool vert = p1.x_ == p2.x_;
        bool needed1 = false;
        bool needed2 = false;
        if (hor || vert) { //strait
            if (vert) {
                needed1 = !viad.test_set(edge.first_);
                needed2 = !viad.test_set(edge.second_);
            }
            dump_segment(netElement, std::forward_as_tuple(p1,p2), needed1, needed2);
        } else {
            auto seg = p1.horizontalSegment(p2);
            dump_segment(netElement, seg);
            needed2 = !viad.test_set(edge.second_);
            dump_segment(netElement, std::forward_as_tuple(seg.second, p2), true, needed2);
        }
        if (!hor) { //vert or 2-seg
            if (cnts_[edge.second_].degree() == 1) {
                std::cout << "Zero segment" << std::endl;
                dump_segment(netElement, std::forward_as_tuple(p2,p2));
            }
        }
    }

private:
    void swap(SteinerMST& other) {
        std::swap(graph_, other.graph_);
        std::swap(edges_, other.edges_);
        std::swap(cnts_, other.cnts_);
        std::swap(length_, other.length_);
    }

public:
   SteinerMST(SteinerGridGraph & gr):
        graph_(gr), cnts_(gr.pts_.size()), length_(-1) {} //defined underflow

    void tell_candidate_size(unsigned new_sz) {
        cnts_.resize(new_sz, NodeConnections());        
    }    
 
   unsigned compute() {
        unsigned total_len = 0;
        edges_.reserve(cnts_.size() - 1);//for example

        //choose Kruscal algorithm for implementation
        std::priority_queue<GridGraphEdge, std::vector<GridGraphEdge>, std::greater<GridGraphEdge>> edges;
        
        typedef std::map<unsigned, size_t> rank_t; // => order on Element
        typedef std::map<unsigned, unsigned> parent_t;
        rank_t rank_map;
        parent_t parent_map;
        boost::associative_property_map<rank_t>   rank_pmap(rank_map);
        boost::associative_property_map<parent_t> parent_pmap(parent_map);
        boost::disjoint_sets<decltype(rank_pmap), decltype(parent_pmap)> dsets(rank_pmap, parent_pmap); 

        unsigned active_num = 0;
        //active->core or in a mask
        for (unsigned cur1 = 0;
            cur1 != graph_.pts_.size();
            cur1 = graph_.pt_next_active(cur1)) 
        {
            active_num++;
            dsets.make_set(cur1);
            cnts_[cur1].resetConnections();
            for (unsigned cur2 = 0;
                 cur2 != cur1;
                 cur2 = graph_.pt_next_active(cur2))
            {
                 edges.emplace(GridGraphEdge(graph_, cur1, cur2));
            }
        }
        while (!edges.empty()) { 
            auto cur = edges.top();
            edges.pop(); 

            unsigned root1 = dsets.find_set(cur.first_);
            unsigned root2 = dsets.find_set(cur.second_);
            if (root1 != root2) { //not connected yet
                total_len += cur.length_;
                dsets.link(root1, root2);
 
                cnts_[cur.first_].connect(edges_.size());
                cnts_[cur.second_].connect(edges_.size());
                edges_.emplace_back(std::move(cur));
            }
        }
        assert(edges_.size() == (active_num - 1));
        length_ = total_len;
        return total_len;
    }   

    unsigned update(unsigned cand_num) {
        graph_.active_.set(cand_num);
        SteinerMST newMST(graph_);
        unsigned len = newMST.compute();
        if (len < length_) {
            swap(newMST);
        } else {
            graph_.active_.reset(cand_num);
        }   
        return length_;
    }

    unsigned ensure_degree() {
        auto & mask = graph_.active_;
        unsigned del_num = 0;
        unsigned mask_cur = 0;
        for (auto cur = cnts_.begin() + graph_.core_pts_num_;
             cur != cnts_.end(); ++cur) {
            if (!mask.test(mask_cur))
                continue;
            if (cur->degree() <= 2) {
                del_num++;
                mask.reset(mask_cur);
            }
            ++mask_cur;
        } 
        return del_num;
    }
};
//TODO consider steiner points in reverse order
//TODO reserve n^2
struct SteinerAlgo {
    SteinerGridGraph & graph_;
    SteinerMST mst_;

private:
    void extend_candidates(unsigned p1_id, unsigned p2_id) {
        auto st_pts = graph_.pts_[p1_id].steinerPoints(graph_.pts_[p2_id]);
        graph_.pts_.push_back(st_pts.first);
        graph_.pts_.push_back(st_pts.second);
    }

    void build_candidates() {
        for (unsigned p_handle = 0;
            p_handle != graph_.core_pts_num_;
            ++p_handle) 
        {
            for (unsigned cur = 0; cur != p_handle; ++cur) 
            {
                extend_candidates(p_handle, cur);
            }
        }
        
        graph_.active_.resize(graph_.cand_pts_num());
        mst_.tell_candidate_size(graph_.pts_.size());
        //may be excessive with respect to amount of points included
        //TODO speed up by excluding points on outer border
    }

    void update_candidates(unsigned chosen) {
        for (unsigned cur = 0; cur < chosen; 
        cur = graph_.pt_next_active(cur)) 
        {
            extend_candidates(cur, chosen);
        }
        graph_.active_.resize(graph_.cand_pts_num());
        mst_.tell_candidate_size(graph_.pts_.size());
        mst_.ensure_degree();
    }

    void choose_candidate (unsigned num_chosen) {
        graph_.active_.set(num_chosen);
        update_candidates(num_chosen);
    }


public:   
    SteinerAlgo(SteinerGridGraph & gr):
        graph_(gr), mst_(gr) {}

    unsigned computeSteinerMST() {
        unsigned cur_length = mst_.compute();
        build_candidates();

        unsigned cand_num = 0;
        unsigned cand_end = graph_.cand_pts_num();
        
        //TODO if candidates were sorted by a heuristic, it would have improved perf
        
        bool progress = true;
        while (progress) {
            progress = false;
            cand_num = 0;
            for (cand_num = 0; cand_num != cand_end; ++cand_num) {
                if (graph_.active_.test(cand_num))
                    continue;
                unsigned new_length = mst_.update(cand_num);
                if (new_length < cur_length) { //included new point
                    choose_candidate(cand_num);
                    cand_end = graph_.pts_.size() - graph_.core_pts_num_;
                    progress = true;
                    cur_length = new_length;
                    break;
                }
            }
        }
        return cur_length;
    }
};

