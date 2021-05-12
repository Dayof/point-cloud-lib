#ifndef __PC_UTILS_HPP__
#define __PC_UTILS_HPP__

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

using namespace std;


struct PointCloud {

    struct Point { float x, y, z; };

    vector< Point > pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Returns false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

PointCloud fromIdxToPointCloud(PointCloud &old_pc, vector<int> &ref_idx);

void readPointCloud(PointCloud &point, string infile);
void savePointCloud(PointCloud &point, string outfile);

#endif // __PC_UTILS_HPP__