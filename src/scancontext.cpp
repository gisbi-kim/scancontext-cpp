#include "scancontext.h"

scancontext::Scancontext::Scancontext
()
: num_rings(20), num_sectors(60), max_len(80.0)
{
    
}

scancontext::Scancontext::Scancontext
(int num_rings_, int num_sectors_, double max_len_)
: num_rings(num_rings_), num_sectors(num_sectors_), max_len(max_len_)
{
    
}

// TBD
void 
scancontext::Scancontext::ptcloud2desc (pointcloud::Pointcloud ptcloud) 
{
    // example case: read .bin to Descriptor
    // - later: from ROS Pointcloud2 topic 

    // NOTE: should add epsilon to the whole element to prevent being devided by zero at distance calc!
    // TBA
}


double norm (std::vector<double> &vec_)
{
    double accum = 0.;
    for(auto & v_: vec_) 
        accum += v_ * v_;
    double norm = sqrt(accum);
    return norm;
}

double mean(std::vector<double> &vec_ )
{
    return accumulate( vec_.begin(), vec_.end(), 0.0) / vec_.size();
}

double cosine_similarity (std::vector<double> a, std::vector<double> b)
{
    double inner_prod = std::inner_product(a.begin(), a.end(), b.begin(), 0.0); // last arg is init accumulation val.
    double cosine_sim = inner_prod / (norm(a) * norm(b));
    return cosine_sim;
}

double scancontext::distance(const Descriptor& sc1, const Descriptor& sc2)
{
    // copy once at first 
    Descriptor sc1_shift;
    sc1_shift.resize(sc1.size());
    std::copy( sc1.begin(), sc1.end(), sc1_shift.begin() );

    // info 
    int num_sectors = sc1.size();
    int num_rings = sc1.at(0).size();
    cout << "ring: " << num_rings << ", sector: " << num_sectors << endl;

    // calc dist by aligning        
    double min_dist{1.0}; // init with max dist 
    int argmin_shift{0};
    for (int shift_idx = 0; shift_idx < num_sectors; ++shift_idx)
    {
        // @ calc current dist        
        std::vector<double> sectors_cossim;
        // sectors_cossim.reserve(num_sectors);         
        std::transform( sc1_shift.begin(), sc1_shift.end(), sc2.begin(), 
                        std::back_inserter(sectors_cossim), // save here 
                        cosine_similarity // two arguments function 
        );
        double cur_cossim = mean(sectors_cossim);              
        double cur_dist = 1.0 - cur_cossim;
        cout << cur_dist << endl;

        // @ check minimal 
        if(cur_dist < min_dist)
        {
            min_dist = cur_dist;
            argmin_shift = shift_idx;

            disp("found!");
            disp("cur min dist: " << min_dist);
            disp("shift idx: " << argmin_shift);
            lc;
        }

        // @ shift sc1
        std::rotate(sc1_shift.rbegin(), sc1_shift.rbegin() + 1, sc1_shift.rend());

    }

    return 1.3;
}


vector<double> eigenvec2stdvec(const VectorXd& eigvec_)
{
    vector<double> stdvec;
    stdvec.reserve(eigvec_.size());
    for (int i = 0; i < eigvec_.size(); ++i)
    {
        stdvec.push_back(eigvec_[i]);
    }
    return stdvec;
}


scancontext::Descriptor eigen2desc(const MatrixXd& eigm_)
{
    int num_rings = eigm_.rows();
    int num_sectors = eigm_.cols();

    scancontext::Descriptor desc;
    desc.reserve(num_sectors);

    for (int sector_idx = 0; sector_idx < num_sectors; ++sector_idx)
    {
        scancontext::Sector sector = eigenvec2stdvec(eigm_.col(sector_idx));
        desc.push_back(sector);
    }

    return desc;
}

void printDesc(const scancontext::Descriptor & desc_)
{
    int sector_idx{1};
    for(auto sector_: desc_)
    {
        cout << "sector " << sector_idx << ": ";
        printVector(sector_);
        sector_idx = sector_idx + 1;
    }
}
