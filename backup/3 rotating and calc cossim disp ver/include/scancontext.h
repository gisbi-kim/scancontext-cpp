#ifndef _SCANCONTEXT_H_
#define _SCANCONTEXT_H_

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
// #include <complex>

#include <Eigen/Dense>

#include "pointcloud.h"
#include "utils.h"

#define lc cout << endl

using namespace std;
using namespace Eigen;

namespace scancontext
{

using Ring = vector<double>;
using Sector = vector<double>;

using Descriptor = vector<Sector>;
using Ringkey = vector<double>;

class Scancontext
{
public:
    Scancontext();
    Scancontext(int, int, double = 80.0);

    void ptcloud2desc(pointcloud::Pointcloud ptcloud); // TBD
    void desc2rk(void); // TBD

public:
    // param default: original paper's setting
    const int num_rings = 20;
    const int num_sectors = 60;
    const double max_len = 80; 

    Descriptor desc;
    Ringkey ringkey;
};


double distance(const Descriptor& sc1, const Descriptor& sc2);

class ScancontextManager
{
    
};


} // namespace ScanContext 

// misc 
scancontext::Descriptor eigen2desc(const MatrixXd& eigm);

template<class T>
void printVector(const std::vector<T> & vec_)
{
    for(auto & v_: vec_)
    {
        cout << v_ << ", ";
    }
    cout << endl;
}

void printDesc(const scancontext::Descriptor & desc_);

#endif // _SCANCONTEXT_H_

