#ifndef HUNGARIAN_HUNGARIAN_H
#define HUNGARIAN_HUNGARIAN_H

#include <map>
#include <tuple>
#include <vector>
#include <cstdint>
#include <cstddef>

#include <Eigen/Core>

std::map<uint32_t, uint32_t>
findAssignment (const std::vector<std::tuple<uint32_t, uint32_t, size_t>>& triples);

std::map<uint32_t, uint32_t>
findAssignment (const Eigen::MatrixXi& weights);

#endif /* HUNGARIAN_HUNGARIAN_H */

