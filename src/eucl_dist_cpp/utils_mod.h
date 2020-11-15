#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <regex>


using namespace std;


template <typename T>
struct PointCloud {

    struct Point { T x, y, z, intensity; };

    vector< Point > pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
};

template <typename T>
void readPointCloud(PointCloud<T> &point, string infile) {
    fstream input(infile.c_str(), ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Could not read file: " << infile << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    int vtx_n;
    // read header
    string line;
    for (int i = 0; input.good() && i <= 7; i++) {
        getline(input, line);
        if (i == 2) {
            // get number of vertexes
            string output;
            smatch m;
            regex rx(R"((?:^|\s)([[:digit:]]+)(?=$|\s))");
            while (regex_search(line, m, rx)) {
                output = m[1];
                break;
            }  
            vtx_n = stoi(output);
            cout << "Point cloud with " << vtx_n << " vertexes" << endl;
        }
    }

    point.pts.resize(vtx_n);
    cout << "PC resized to " << vtx_n << endl;
    for (size_t i = 0; input.good() && !input.eof(); i++) {
        getline(input, line);   
        istringstream in(line);
        if (!input.eof()) in >> point.pts[i].x >> point.pts[i].y >> point.pts[i].z >> point.pts[i].intensity;
    }
    input.close();

    cout << "Loaded PC of size " << point.kdtree_get_point_count() 
         << " from file " << infile << endl;
}

template <typename T>
vector <int> getPointsInnov(PointCloud<T> &point, vector <int> &intersec_vec) {
    vector <int> total_vec, diff;

    for (size_t i = 0; i <= point.kdtree_get_point_count(); ++i)
        total_vec.push_back(i);

    set_difference(total_vec.begin(), total_vec.end(),
                   intersec_vec.begin(), intersec_vec.end(),
                   inserter(diff, diff.begin()));

    return diff;
}

template <typename T>
void savePointCloud(PointCloud<T> &point, vector <int> &intersec_vec, string outfile) {
    fstream output(outfile.c_str(), ios::out | ios::binary);
    if (!output.good()) {
        cerr << "Could not read file: " << outfile << endl;
        exit(EXIT_FAILURE);
    }

    output << "ply" << endl;
    output << "format ascii 1.0" << endl;
    output << "element vertex " << intersec_vec.size() << endl;
    output << "property float x" << endl;
    output << "property float y" << endl;
    output << "property float z" << endl;
    output << "property float reflect_coeff" << endl;
    output << "end_header" << endl;

    for ( auto idx: intersec_vec ) {
        output << point.kdtree_get_pt(idx, 0) << " " 
                << point.kdtree_get_pt(idx, 1) << " "
                << point.kdtree_get_pt(idx, 2) << " " 
                << point.kdtree_get_pt(idx, 3) << endl;
    }

    output.close();

}