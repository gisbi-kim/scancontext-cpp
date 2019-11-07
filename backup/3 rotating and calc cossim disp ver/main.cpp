#include <iostream>
#include <string>
#include <memory>

#include <Eigen/Dense>

#include "utils.h"
#include "scancontext.h"
#include "pointcloud.h"

using std::cout;
using std::endl;
using std::string;
using namespace Eigen;

using namespace utils;
using SC = scancontext::Scancontext;

#define lc cout << endl

auto main (void) -> int 
{
    // @ start 
    auto disp_start = [](string str_){
        auto str = std::make_unique<string>(str_);
        cout << *str << endl;
        disp(*str + "(printed by using utils)");
    }; 
    disp_start("Scan Context test Starts"); lc;

    // @ make
    // int num_rings{40}; 
    // int num_sectors{120};
    int num_rings{4}; 
    int num_sectors{12};
    SC sc1(num_rings, num_sectors);
    
    disp(sc1.num_rings);    
    disp(sc1.num_sectors);    
    disp(sc1.max_len);    
    lc;

    // @ Test 1
    MatrixXd sc_const = MatrixXd::Constant(num_rings, num_sectors, 2.0);
    {
        MatrixXd sc_A0_eig = MatrixXd::Random(num_rings, num_sectors) + sc_const;
        cout << sc_A0_eig << endl;
        lc; 

        scancontext::Descriptor sc_A0 = eigen2desc(sc_A0_eig);
        printDesc(sc_A0);
        lc;
    }

    // @ Test 2: distance
    MatrixXd sc_A0_eig = MatrixXd::Random(num_rings, num_sectors) + sc_const;
    scancontext::Descriptor sc_A0 = eigen2desc(sc_A0_eig);

    MatrixXd sc_B0_eig = MatrixXd::Random(num_rings, num_sectors) + sc_const;
    scancontext::Descriptor sc_B0 = eigen2desc(sc_B0_eig);

    cout << "distance btn A0 and B0 is: " << scancontext::distance(sc_A0, sc_B0) << endl;


    return 0;
}
