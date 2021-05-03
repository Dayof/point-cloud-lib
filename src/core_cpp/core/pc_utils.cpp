#include "pc_utils.hpp"

#include <algorithm>
#include <iterator>
#include <fstream>
#include <sstream>
#include <regex>

using namespace std;


void readPointCloud(PointCloud &point, string infile) {
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
        if (i == 3) {
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
        if (!input.eof()) in >> point.pts[i].x >> point.pts[i].y >> point.pts[i].z;
        // if (!input.eof()) in >> point.pts[i].x >> point.pts[i].y >> point.pts[i].z >> point.pts[i].intensity;
    }
    input.close();

    cout << "Loaded PC of size " << point.kdtree_get_point_count() 
         << " from file " << infile << endl << endl;
}


vector <int> getPointsInnov(PointCloud &point, vector <int> &intersec_vec) {
    vector <int> total_vec, diff;

    for (size_t i = 0; i <= point.kdtree_get_point_count(); ++i)
        total_vec.push_back(i);

    set_difference(total_vec.begin(), total_vec.end(),
                   intersec_vec.begin(), intersec_vec.end(),
                   inserter(diff, diff.begin()));

    return diff;
}


PointCloud fromIdxToPointCloud(PointCloud &old_pc, vector<int> &ref_idx) {
    PointCloud new_pc;
    new_pc.pts.reserve(ref_idx.size());
    for (size_t idx = 0; idx <= ref_idx.size(); ++idx) {
        if (ref_idx[idx]) {
            // cout << "point: " << idx << endl;
            PointCloud::Point pts_vec;
            pts_vec.x = old_pc.kdtree_get_pt(idx, 0);
            pts_vec.y = old_pc.kdtree_get_pt(idx, 1);
            pts_vec.z = old_pc.kdtree_get_pt(idx, 2);
            // pts_vec.intensity = old_pc.kdtree_get_pt(idx, 3);
            new_pc.pts.push_back(pts_vec);
        }
    }
    return new_pc;
}


void savePointCloud(PointCloud &point, string outfile) {
    fstream output(outfile.c_str(), ios::out | ios::binary);
    if (!output.good()) {
        cerr << "Could not read file: " << outfile << endl;
        exit(EXIT_FAILURE);
    }

    output << "ply" << endl;
    output << "format ascii 1.0" << endl;
    output << "element vertex " << point.kdtree_get_point_count() << endl;
    output << "property float x" << endl;
    output << "property float y" << endl;
    output << "property float z" << endl;
    // output << "property float reflect_coeff" << endl;
    output << "end_header" << endl;

    for ( auto local_point: point.pts ) {
        output << local_point.x << " " 
                << local_point.y << " "
                << local_point.z << endl;
    }

    output.close();
}
