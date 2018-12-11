#include "tinyxml2.h"
#include "steiner.h"
#include <string>

template <typename InsIter>
void get_nodes(InsIter it, const char * filename) {
    using namespace tinyxml2; 

    XMLDocument doc;
    auto err_code = doc.LoadFile(filename);

    if (err_code != XML_SUCCESS) {
        fprintf(stderr, "File cannot be loaded. Probably the path is wrong\n");
        exit(-1);
    }
    XMLElement* gridElement = doc.FirstChildElement()->FirstChildElement( "grid" );
    int min_x, min_y;
    gridElement->QueryIntAttribute( "min_x", &min_x );
    gridElement->QueryIntAttribute( "min_y", &min_y );
    assert(min_x >= 0 && min_y >= 0);

    XMLElement * netElement = doc.FirstChildElement()->FirstChildElement("net");

    XMLElement * cur_pt_node = netElement->FirstChildElement("point");
    for (; cur_pt_node != nullptr; cur_pt_node = cur_pt_node->NextSiblingElement()) {
        unsigned x, y;
        x = cur_pt_node->UnsignedAttribute("x");
        y = cur_pt_node->UnsignedAttribute("y");
        *(it++) = GridGraphNode(x,y);
    }
}

void dump_edges(tinyxml2::XMLElement * netElement, SteinerMST & solution) {
    using namespace tinyxml2;

    auto & pts = solution.graph_.pts_;
    for (auto& edge: solution.edges_) {
        printf("(%u, %u)->(%u, %u)\n", 
                pts[edge.first_].x_,
                pts[edge.first_].y_,
                pts[edge.second_].x_,
                pts[edge.second_].y_
        );
        solution.dump_edge(netElement, edge);
    }
}

void print_answer(SteinerMST & solution, char * argv[]) {
    using namespace tinyxml2; 
    
    XMLDocument doc;
    std::string to = argv[1];
    auto index = to.rfind(".xml");
    to.insert(index, "_out");
    doc.LoadFile(argv[1]);
    XMLElement * netElement = doc.FirstChildElement()->FirstChildElement("net");

    auto & pts = solution.graph_.pts_;
    for (auto cur_node = pts.begin(); cur_node != pts.begin() + solution.graph_.core_pts_num_; ++cur_node) {
        cur_node->dump_via(netElement); 
//if point is a steiner point, it has a degree > 2 and have via automatically when printing edges
//core pts itself are printed in initial doc
    }
    dump_edges(netElement, solution);    
    doc.SaveFile(to.c_str());
}

int main(int argc, char* argv[]) {
    //first argument is npoints, then b
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <file_in>.xml\nThe result placed in <file_in>_out.xml\n", argv[0]);
        exit(0);
    }
    
    std::vector<GridGraphNode> a;
    get_nodes(std::back_inserter(a), argv[1]);

    SteinerGridGraph graph(std::begin(a), std::end(a)); 
    SteinerAlgo algorithm(graph);
    std::cout << "Steiner MST len is " << algorithm.computeSteinerMST() << std::endl;

    print_answer(algorithm.mst_, argv);
    return 0;
}

