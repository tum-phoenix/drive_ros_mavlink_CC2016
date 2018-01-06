#ifndef CALC_COV_H
#define CALC_COV_H

#include <cmath>
#include <vector>


// calculates the covariances based on the current velocity
double calculateCovariance(const std::vector<double> coef, const double vel)
{
  double out = 0;
  for(int i=0; i<coef.size(); i++)
  {
    out += coef.at(coef.size()-1-i) * std::pow(vel, i);
  }

  // this should never happen, but try to avoid values below 0 anyway
  return std::max(out, double(0));
}
#endif // CALC_COV_H
