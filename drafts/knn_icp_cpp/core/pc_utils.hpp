#ifndef __PC_UTILS_HPP__
#define __PC_UTILS_HPP__

#include <iostream>
#include <cstdlib>
#include <vector>
#include <queue>

using namespace std;


struct PointCloud {

    struct Point { float x, y, z, xy, yz, xz, intensity; };

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

struct PCQueue {

    queue<pair<int, PointCloud>> pc_refs;

    void push(int index, PointCloud &point) {
        pair <int, PointCloud> new_pc_ref (index, point);
        pc_refs.push(new_pc_ref);
        cout << "PC " << index << " added to queue reference." << endl;
    }

    PointCloud points() {
        queue<pair<int, PointCloud>> cur_pc_ref = pc_refs;
        PointCloud ref_pc;
        while (!cur_pc_ref.empty()) {
            vector<PointCloud::Point> pc_points = cur_pc_ref.front().second.pts;
            cout << "PC size: " << pc_points.size() << endl;
            cout << "Resizing PCQueue from " << ref_pc.pts.size()
                 << " to " << pc_points.size() << endl;
            size_t last_idx = ref_pc.pts.size();
            ref_pc.pts.resize(ref_pc.pts.size() + pc_points.size());
            for (size_t i = last_idx; i <= ref_pc.pts.size(); ++i) {
                ref_pc.pts[i].x = pc_points[i].x;
                ref_pc.pts[i].y = pc_points[i].y;
                ref_pc.pts[i].z = pc_points[i].z;
                ref_pc.pts[i].intensity = pc_points[i].intensity;
            }
            cur_pc_ref.pop();
        }
        cout << "PC ref with " << ref_pc.kdtree_get_point_count() << " points." << endl;
        return ref_pc;
    }
};

vector <int> getPointsInnov(PointCloud &point, vector <int> &intersec_vec);
PointCloud fromIdxToPointCloud(PointCloud &old_pc, vector<int> &ref_idx);

void readPointCloud(PointCloud &point, string infile);
void savePointCloud(PointCloud &point, string outfile);

#endif // __PC_UTILS_HPP__