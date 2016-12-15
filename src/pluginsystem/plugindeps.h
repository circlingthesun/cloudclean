#include <pcl/search/kdtree.h>
#include <pcl/segmentation/boost.h>
#include <pcl/search/search.h>
#include <pcl/common/pca.h>
#include <cmath>

typedef boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::directedS > Traits;

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::directedS,
                             boost::property< boost::vertex_name_t, std::string,
                               boost::property< boost::vertex_index_t, long,
                                 boost::property< boost::vertex_color_t, boost::default_color_type,
                                   boost::property< boost::vertex_distance_t, long,
                                     boost::property< boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,
                             boost::property< boost::edge_capacity_t, double,
                               boost::property< boost::edge_residual_capacity_t, double,
                                 boost::property< boost::edge_reverse_t, Traits::edge_descriptor > > > > mGraph;

typedef boost::property_map< mGraph, boost::edge_capacity_t >::type CapacityMap;

typedef boost::property_map< mGraph, boost::edge_reverse_t>::type ReverseEdgeMap;

typedef Traits::vertex_descriptor VertexDescriptor;

typedef boost::graph_traits< mGraph >::edge_descriptor EdgeDescriptor;

typedef boost::graph_traits< mGraph >::out_edge_iterator OutEdgeIterator;

typedef boost::graph_traits< mGraph >::vertex_iterator VertexIterator;

typedef boost::property_map< mGraph, boost::edge_residual_capacity_t >::type ResidualCapacityMap;

typedef boost::property_map< mGraph, boost::vertex_index_t >::type IndexMap;

typedef boost::graph_traits< mGraph >::in_edge_iterator InEdgeIterator;


void linkerhack(){
    pcl::search::KdTree<pcl::PointXYZI> wee;
    pcl::search::Search <pcl::PointXYZI> * KdTree;
    boost::shared_ptr<mGraph> graph_;
    graph_ = boost::shared_ptr< mGraph > (new mGraph ());
    pcl::PCA<pcl::PointXYZI> pc(false);
}
