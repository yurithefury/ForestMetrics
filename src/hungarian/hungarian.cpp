#include "hungarian.h"

#include "Assignment.h"
#include "Hungarian.h"
#include "CmdParser.h"
#include "BipartiteGraph.h"

std::map<uint32_t, uint32_t>
findAssignment (const std::vector<std::tuple<uint32_t, uint32_t, size_t>>& triples)
{
  std::map<uint32_t, size_t> l1_map;
  std::map<size_t, uint32_t> l1_map_inverse;
  std::map<uint32_t, size_t> l2_map;
  std::map<size_t, uint32_t> l2_map_inverse;
  // Iterate over triples and build a label mapping
  uint32_t l1;
  uint32_t l2;
  size_t c;
  for (size_t i = 0; i < triples.size (); ++i)
  {
    std::tie(l1, l2, c) = triples[i];
    if (l1_map.count (l1) == 0)
    {
      size_t i = l1_map.size ();
      l1_map.insert (std::make_pair (l1, i));
      l1_map_inverse.insert (std::make_pair (i, l1));
    }
    if (l2_map.count (l2) == 0)
    {
      size_t i = l2_map.size ();
      l2_map.insert (std::make_pair (l2, i));
      l2_map_inverse.insert (std::make_pair (i, l2));
    }
  }
  // Create matrix of proper size
  Matrix m (l1_map.size ());
  for (size_t i = 0; i < l1_map.size (); ++i)
    m[i].resize (l2_map.size ());
  // Fill weights
  for (size_t i = 0; i < triples.size (); ++i)
  {
    std::tie(l1, l2, c) = triples[i];
    m[l1_map[l1]][l2_map[l2]].SetWeight (c);
  }
  // Run the algorithm
  std::map<uint32_t, uint32_t> assignment;
  BipartiteGraph bg (m);
  Hungarian h (bg);
  h.HungarianAlgo ();
  // Create assignment
  for (const auto& pair : h.M)
    assignment.insert (std::make_pair (l1_map_inverse[pair.first], l2_map_inverse[pair.second]));
  return assignment;
}

std::map<uint32_t, uint32_t>
findAssignment (const Eigen::MatrixXi& weights)
{
  bool transpose = weights.rows () > weights.cols ();
  int rows = !transpose ? weights.rows () : weights.cols ();
  int cols = !transpose ? weights.cols () : weights.rows ();

  Matrix m (rows);
  for (int i = 0; i < rows; ++i)
  {
    m[i].resize (cols);
    for (int j = 0; j < cols; ++j)
      m[i][j].SetWeight (!transpose ? weights (i, j) : weights (j, i));
  }
  // Run the algorithm
  std::map<uint32_t, uint32_t> assignment;
  BipartiteGraph bg (m);
  Hungarian h (bg);
  h.HungarianAlgo ();
  // Create assignment
  for (const auto& pair : h.M)
    assignment.insert (std::make_pair (!transpose ? pair.first : pair.second,
                                       !transpose ? pair.second : pair.first));
  return assignment;
}

