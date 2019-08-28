#ifndef SCALER_H
#define SCALER_H

#include <algorithm>

#include <Eigen/Core>

typedef std::function<float (float)> Scaler;

Scaler getScaler (float min, float max)
{
  if (!std::isfinite (min) || !std::isfinite (max))
    throw std::runtime_error ("requested scaler for collection with infinite elements");
  if (max != min)
    return [min, max] (float v) { return (v - min) / (max - min); };
  else
    return [] (float) { return 1.0; };
}

template <typename T>
Scaler getScaler (const T& a)
{
  float min = a.min ();
  float max = a.max ();
  return getScaler (min, max);
}

template <>
Scaler getScaler (const Eigen::VectorXf& a)
{
  float min = a.minCoeff ();
  float max = a.maxCoeff ();
  return getScaler (min, max);
}

template <>
Scaler getScaler (const std::vector<float>& a)
{
  float min = *std::min_element (a.begin (), a.end ());
  float max = *std::max_element (a.begin (), a.end ());
  return getScaler (min, max);
}

#endif /* SCALER_H */

