#include "mincut.h"

#include <pcl/segmentation/boost.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <stdlib.h>
#include <cmath>
#include <QDebug>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MinCut::MinCut () :
  inverse_sigma_ (16.0),
  binary_potentials_are_valid_ (false),
  epsilon_ (0.0001),
  radius_ (16.0),
  unary_potentials_are_valid_ (false),
  source_weight_ (0.8),
  search_ (),
  number_of_neighbours_ (14),
  graph_is_valid_ (false),
  //foreground_points_ (0),
  //background_points_ (0),
  clusters_ (0),
  graph_ (),
  capacity_ (),
  reverse_edges_ (),
  vertices_ (0),
  edge_marker_ (0),
  source_ (),/////////////////////////////////
  sink_ (),///////////////////////////////////
  max_flow_ (0.0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MinCut::~MinCut ()
{
  if (search_ != 0)
    search_.reset ();
  if (graph_ != 0)
    graph_.reset ();
  if (capacity_ != 0)
    capacity_.reset ();
  if (reverse_edges_ != 0)
    reverse_edges_.reset ();

  foreground_points_.clear ();
  background_points_.clear ();
  clusters_.clear ();
  vertices_.clear ();
  edge_marker_.clear ();
  input_.reset ();
  indices_.reset ();
}

boost::shared_ptr<MinCut::gData> MinCut::getGraphData(){
    qDebug("get graph data called!");
    boost::shared_ptr<MinCut::gData> data = boost::shared_ptr<MinCut::gData>(new MinCut::gData());

    // Finds all points that are indexed
    std::vector<int> labels;
    labels.resize (input_->points.size (), 0);
    int number_of_indices = static_cast<int> (indices_->size ());
    for (int i_point = 0; i_point < number_of_indices; i_point++)
        labels[(*indices_)[i_point]] = 1;

    // Note: Only valid after extract was run
    ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

    // temporary vertices
    std::vector<int> tmp_vertices;
    std::vector<int> tmp_labels;

    // Set vertices
    OutEdgeIterator edge_iter, edge_end;
    for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
    {
      if (labels[edge_iter->m_target] == 1){
        if (residual_capacity[*edge_iter] > epsilon_){
            data->source_vertices.push_back(edge_iter->m_target);
            tmp_labels.push_back(0);
        }
        else{
            data->sink_vertices.push_back(edge_iter->m_target);
            tmp_labels.push_back(1);
        }
        tmp_vertices.push_back(edge_iter->m_target);
      }
    }


    // Set up edges

    // checks for duplicates
    std::set<std::pair<int, int> > edge_marker;

    // Find maximum edge weight

    float max_weight = 0;

    for(int idx : tmp_vertices){
        // For every neighbour edge
        for ( boost::tie (edge_iter, edge_end) = boost::out_edges (idx, *graph_); edge_iter != edge_end; edge_iter++ ) {
            VertexDescriptor target = edge_iter->m_target;
            if(target == source_ || target == sink_)
                continue;

            // Set up edge
            int a = idx, b = static_cast<int>(target);
            assert(a !=b);
            if(a > b){ int tmp = a; a = b; b = tmp;}
            std::pair<int, int> edge = std::make_pair(a, b);
            auto retpair = edge_marker.insert(edge);

            // Continue the egde was previously inserted
            if(!retpair.second)
                continue;            

            float weight = residual_capacity[*edge_iter];

            if(weight > max_weight)
                max_weight = weight;

            // Determine label
            if(tmp_labels[idx] == tmp_labels[static_cast<int>(target)]){
                if(tmp_labels[idx] == 0){
                    data->source_edges.push_back(edge);
                    data->source_edge_weights.push_back(weight);
                }
                else{
                    data->sink_edges.push_back(edge);
                    data->sink_edge_weights.push_back(weight);
                }
            }
            else{
                data->bridge_edges.push_back(edge);
                data->bridge_edge_weights.push_back(weight);
            }

            // what about the flow?
        }
    }

    /*for(int i = 0; i < data->source_edge_weights.size(); i++)
        qDebug("source weight : %f", data->source_edge_weights[i]);
    for(int i = 0; i < data->sink_edge_weights.size(); i++)
        qDebug("sink weight : %f", data->sink_edge_weights[i]);
    for(int i = 0; i < data->bridge_edge_weights.size(); i++)
        qDebug("bridge weight : %f", data->bridge_edge_weights[i]);
    */

    // normalise weights
    for(uint i = 0; i < data->source_edge_weights.size(); i++)
        data->source_edge_weights[i] /= max_weight;
    for(uint i = 0; i < data->sink_edge_weights.size(); i++)
        data->sink_edge_weights[i] /= max_weight;
    for(uint i = 0; i < data->bridge_edge_weights.size(); i++)
        data->bridge_edge_weights[i] /= max_weight;



    return data;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setInputCloud (PointCloud::Ptr &cloud)
{
  input_ = cloud;
  graph_is_valid_ = false;
  unary_potentials_are_valid_ = false;
  binary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getSigma () const
{
  return (pow (1.0 / inverse_sigma_, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setSigma (double sigma)
{
  if (sigma > epsilon_)
  {
    inverse_sigma_ = 1.0 / (sigma * sigma);
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getRadius () const
{
  return (pow (radius_, 0.5));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setRadius (double radius)
{
  if (radius > epsilon_)
  {
    radius_ = radius * radius;
    unary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getSourceWeight () const
{
  return (source_weight_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setSourceWeight (double weight)
{
  if (weight > epsilon_)
  {
    source_weight_ = weight;
    unary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MinCut::KdTreePtr
MinCut::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setSearchMethod (const KdTreePtr& tree)
{
  if (search_ != 0)
    search_.reset ();

  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 unsigned int
MinCut::getNumberOfNeighbours () const
{
  return (number_of_neighbours_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setNumberOfNeighbours (unsigned int neighbour_number)
{
  if (number_of_neighbours_ != neighbour_number && neighbour_number != 0)
  {
    number_of_neighbours_ = neighbour_number;
    graph_is_valid_ = false;
    unary_potentials_are_valid_ = false;
    binary_potentials_are_valid_ = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 std::set<int> MinCut::getForegroundPoints() const
{
  return (foreground_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setForegroundPoints (std::set<int> &foreground_points)
{
  foreground_points_.clear ();
  for (int i : foreground_points)
    foreground_points_.insert(i);

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 std::set<int> MinCut::getBackgroundPoints() const
{
  return (background_points_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::setBackgroundPoints (std::set<int> &background_points)
{
  background_points_.clear ();
  for (int i: background_points)
    background_points_.insert(i);

  unary_potentials_are_valid_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  // copy was here

  clusters_.clear ();
  bool success = true;

  if ( !graph_is_valid_ )
  {
    success = buildGraph ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    graph_is_valid_ = true;
    unary_potentials_are_valid_ = true;
    binary_potentials_are_valid_ = true;
  }

  if ( !unary_potentials_are_valid_ )
  {
    success = recalculateUnaryPotentials ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    unary_potentials_are_valid_ = true;
  }

  if ( !binary_potentials_are_valid_ )
  {
    success = recalculateBinaryPotentials ();
    if (success == false)
    {
      deinitCompute ();
      return;
    }
    binary_potentials_are_valid_ = true;
  }

  // All checks are done here

  //IndexMap index_map = boost::get (boost::vertex_index, *graph_);
  ResidualCapacityMap residual_capacity = boost::get (boost::edge_residual_capacity, *graph_);

  max_flow_ = boost::boykov_kolmogorov_max_flow (*graph_, source_, sink_);

  assembleLabels (residual_capacity);

  // Copy current clusters to input ref?? WHY?
  if ( graph_is_valid_ && unary_potentials_are_valid_ && binary_potentials_are_valid_ )
  {
    //clusters.reserve (clusters_.size ());
    std::copy (clusters_.begin (), clusters_.end (), std::back_inserter (clusters));
    //getClusters(clusters);
    deinitCompute ();
    return;
  }

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::getMaxFlow () const
{
  return (max_flow_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  boost::shared_ptr< MinCut::mGraph>
MinCut::getGraph () const
{
  return (graph_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool
MinCut::buildGraph ()
{
  int number_of_points = static_cast<int> (input_->points.size ());
  int number_of_indices = static_cast<int> (indices_->size ());

  if (input_->points.size () == 0 || number_of_points == 0 || foreground_points_.empty () == true )
    return (false);

  if (search_ == 0)
    search_ = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI> > (new pcl::search::KdTree<pcl::PointXYZI>);

  graph_.reset ();
  graph_ = boost::shared_ptr< mGraph > (new mGraph ());

  // Checks done, empty graph

  capacity_.reset ();
  capacity_ = boost::shared_ptr<CapacityMap> (new CapacityMap ());
  *capacity_ = boost::get (boost::edge_capacity, *graph_);

  // Capacity map configured

  reverse_edges_.reset ();
  reverse_edges_ = boost::shared_ptr<ReverseEdgeMap> (new ReverseEdgeMap ());
  *reverse_edges_ = boost::get (boost::edge_reverse, *graph_);

  // Reverse edge map configured

  VertexDescriptor vertex_descriptor(0);
  vertices_.clear ();
  vertices_.resize (number_of_points + 2, vertex_descriptor);

  // Added 2 new vertices with 0 descriptors

  std::set<int> out_edges_marker;
  edge_marker_.clear ();
  edge_marker_.resize (number_of_points + 2, out_edges_marker);

  // Added two new out edge markers? avoid duplicate adds

  // Add vertices who have default indices (I assume)
  for (int i_point = 0; i_point < number_of_points + 2; i_point++)
    vertices_[i_point] = boost::add_vertex (*graph_);

  // Last two indices are the source and sink
  source_ = vertices_[number_of_points];
  sink_ = vertices_[number_of_points + 1];

  // Link up every point to the source and sink, with weights
  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    double source_weight = 0.0; // Foreground penalty
    double sink_weight = 0.0; // Background penalty
    calculateUnaryPotential (point_index, source_weight, sink_weight);
    addEdge (static_cast<int> (source_), point_index, source_weight); // Connect to source
    addEdge (point_index, static_cast<int> (sink_), sink_weight); // Connect to sink
  }

  // Set the neighbours for every point
  // These are binary edges with weights
  std::vector<int> neighbours;
  std::vector<float> distances;
  search_->setInputCloud (input_, indices_);
  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    search_->nearestKSearch (i_point, number_of_neighbours_, neighbours, distances);
    for (size_t i_nghbr = 1; i_nghbr < neighbours.size (); i_nghbr++) // WHY skip the first neighbour?
    {
      double weight = calculateBinaryPotential (point_index, neighbours[i_nghbr]);
      addEdge (point_index, neighbours[i_nghbr], weight);
      addEdge (neighbours[i_nghbr], point_index, weight);
    }
    neighbours.clear ();
    distances.clear ();
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::calculateUnaryPotential (int point, double& source_weight, double& sink_weight) const
{
  // Given an abritrary point in the cloud.

  // Pin fg and bg points to the source and sink
/*
  // Is this point a foreground point
  for (int fgp : foreground_points_) {
      if(fgp == point){
          sink_weight = 0.5;
          //source_weight = std::numeric_limits<double>::max ();
          source_weight = 0.6;
          return;
      }
  }

  // Is this point a background point
  for (int bgp : background_points_) {
      if(bgp == point){
          //sink_weight = std::numeric_limits<double>::max ();
          sink_weight = 0.5;
          source_weight = 0.5;
          return;
      }
  }
*/
  // If the point is not pinnned

  // Apply background penalty
  sink_weight = 20*number_of_neighbours_;

  // Apply forground penalty
  source_weight = 20*number_of_neighbours_;

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // input cloud indices
 bool
MinCut::addEdge (int source, int target, double weight)
{
  std::set<int>::iterator iter_out = edge_marker_[source].find (target);
  if ( iter_out != edge_marker_[source].end () )
    return (false);

  EdgeDescriptor edge;
  EdgeDescriptor reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( vertices_[source], vertices_[target], *graph_ );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( vertices_[target], vertices_[source], *graph_ );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);

  (*capacity_)[edge] = weight;
  (*capacity_)[reverse_edge] = 0.0;
  (*reverse_edges_)[edge] = reverse_edge;
  (*reverse_edges_)[reverse_edge] = edge;
  edge_marker_[source].insert (target);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 double
MinCut::calculateBinaryPotential (int source, int target) const
{

    bool same_label = foreground_points_.find(source) == foreground_points_.find(target);

    if(same_label)
        return 1;

    double weight = 0.0;
    double distance = 0.0;
    const pcl::PointXYZI & s = input_->points[source];
    const pcl::PointXYZI & t = input_->points[target];
    distance = (s.getVector3fMap() - t.getVector3fMap()).squaredNorm();
    distance *= inverse_sigma_;

    /*if(distance < 1e-05){
        qDebug() << "Nope nop nop nope!";
    }*/

    weight = exp(-distance);

    weight *= 0.5;

    return (weight);

    //return 0.1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool
MinCut::recalculateUnaryPotentials ()
{
  OutEdgeIterator src_edge_iter;
  OutEdgeIterator src_edge_end;
  std::pair<EdgeDescriptor, bool> sink_edge;

  // Itterate though all the edges
  // Set weights to the sink. Set weights on other edges
  // Seems like every node is connected to the source
  for (boost::tie (src_edge_iter, src_edge_end) = boost::out_edges (source_, *graph_); src_edge_iter != src_edge_end; src_edge_iter++)
  {
    double source_weight = 0.0;
    double sink_weight = 0.0;
    sink_edge.second = false;
    calculateUnaryPotential (static_cast<int> (boost::target (*src_edge_iter, *graph_)), source_weight, sink_weight);
    // Lookup the edge from the current edge target to the sink
    sink_edge = boost::lookup_edge (boost::target (*src_edge_iter, *graph_), sink_, *graph_);
    // does this edge does not exist?
    if (!sink_edge.second)
      return (false);

    // Set edge weights
    (*capacity_)[*src_edge_iter] = source_weight; // source to target weight
    (*capacity_)[sink_edge.first] = sink_weight; // target to sink weight
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool
MinCut::recalculateBinaryPotentials ()
{
  int number_of_points = static_cast<int> (indices_->size ());

  VertexIterator vertex_iter;
  VertexIterator vertex_end;
  OutEdgeIterator edge_iter;
  OutEdgeIterator edge_end;

  std::vector< std::set<VertexDescriptor> > edge_marker;
  std::set<VertexDescriptor> out_edges_marker;
  edge_marker.clear ();
  edge_marker.resize (number_of_points + 2, out_edges_marker);

  for (boost::tie (vertex_iter, vertex_end) = boost::vertices (*graph_); vertex_iter != vertex_end; vertex_iter++)
  {
    VertexDescriptor source_vertex = *vertex_iter;
    if (source_vertex == source_ || source_vertex == sink_)
      continue;
    for (boost::tie (edge_iter, edge_end) = boost::out_edges (source_vertex, *graph_); edge_iter != edge_end; edge_iter++)
    {
      //If this is not the edge of the graph, but the reverse fictitious edge that is needed for the algorithm then continue
      EdgeDescriptor reverse_edge = (*reverse_edges_)[*edge_iter];
      if ((*capacity_)[reverse_edge] != 0.0)
        continue;

      //If we already changed weight for this edge then continue
      VertexDescriptor target_vertex = boost::target (*edge_iter, *graph_);
      std::set<VertexDescriptor>::iterator iter_out = edge_marker[static_cast<int> (source_vertex)].find (target_vertex);
      if ( iter_out != edge_marker[static_cast<int> (source_vertex)].end () )
        continue;

      if (target_vertex != source_ && target_vertex != sink_)
      {
        //Change weight and remember that this edges were updated
        double weight = calculateBinaryPotential (static_cast<int> (target_vertex), static_cast<int> (source_vertex));
        (*capacity_)[*edge_iter] = weight;
        edge_marker[static_cast<int> (source_vertex)].insert (target_vertex);
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 void
MinCut::assembleLabels (ResidualCapacityMap& residual_capacity)
{
  std::vector<int> labels;
  labels.resize (input_->points.size (), 0);
  int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; i_point++)
    labels[(*indices_)[i_point]] = 1;

  clusters_.clear ();

  pcl::PointIndices segment;
  clusters_.resize (2, segment);

  OutEdgeIterator edge_iter, edge_end;
  for ( boost::tie (edge_iter, edge_end) = boost::out_edges (source_, *graph_); edge_iter != edge_end; edge_iter++ )
  {
    if (labels[edge_iter->m_target] == 1)
    {
      if (residual_capacity[*edge_iter] > epsilon_)
        clusters_[1].indices.push_back (static_cast<int> (edge_iter->m_target));
      else
        clusters_[0].indices.push_back (static_cast<int> (edge_iter->m_target));
    }
  }
}

void
MinCut::getClusters(std::vector <pcl::PointIndices> & clusters){
    // assuming that clusters_ contain indexes in the graph
    // converting them into cloud indices
    pcl::PointIndices segment;
    clusters.resize (2, segment);
    for(int i : clusters_[0].indices){
        clusters[0].indices.push_back(i);
    }

    for(int i : clusters_[1].indices){
        clusters[1].indices.push_back(i);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr
MinCut::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    int num_of_pts_in_first_cluster = static_cast<int> (clusters_[0].indices.size ());
    int num_of_pts_in_second_cluster = static_cast<int> (clusters_[1].indices.size ());
    int number_of_points = num_of_pts_in_first_cluster + num_of_pts_in_second_cluster;
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
    unsigned char foreground_color[3] = {255, 255, 255};
    unsigned char background_color[3] = {255, 0, 0};
    colored_cloud->width = number_of_points;
    colored_cloud->height = 1;
    colored_cloud->is_dense = input_->is_dense;

    pcl::PointXYZRGB point;
    int point_index = 0;
    for (int i_point = 0; i_point < num_of_pts_in_first_cluster; i_point++)
    {
      point_index = clusters_[0].indices[i_point];
      point.x = *(input_->points[point_index].data);
      point.y = *(input_->points[point_index].data + 1);
      point.z = *(input_->points[point_index].data + 2);
      point.r = background_color[0];
      point.g = background_color[1];
      point.b = background_color[2];
      colored_cloud->points.push_back (point);
    }

    for (int i_point = 0; i_point < num_of_pts_in_second_cluster; i_point++)
    {
      point_index = clusters_[1].indices[i_point];
      point.x = *(input_->points[point_index].data);
      point.y = *(input_->points[point_index].data + 1);
      point.z = *(input_->points[point_index].data + 2);
      point.r = foreground_color[0];
      point.g = foreground_color[1];
      point.b = foreground_color[2];
      colored_cloud->points.push_back (point);
    }
  }

  return (colored_cloud);
}

 bool
 MinCut::deinitCompute ()
 {
   // Reset the indices
  if (fake_indices_)
   {
     indices_.reset ();
     fake_indices_ = false;
   }
   return (true);
 }

 bool
MinCut::initCompute ()
 {
   // Check if input was set
   if (!input_)
     return (false);

   // If no point indices have been given, construct a set of indices for the entire input point cloud
   if (!indices_)
   {
     fake_indices_ = true;
     std::vector<int> *indices = new std::vector<int> (input_->width * input_->height);
     for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }
     indices_.reset (indices);
   }
   return (true);
 }
