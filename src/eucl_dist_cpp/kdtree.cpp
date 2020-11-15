#include "nanoflann.hpp"
#include "utils_mod.h"

#include <ctime>
#include <cstdlib>
#include <iostream>

using namespace std;
using namespace nanoflann;

template <typename num_t>
void kdtree_search() {
	PointCloud<num_t> cloud_1, cloud_2;
	string infile_1 = "/home/dayoff/codes/point_cloud_lib/data/kitti/0000000000.ply";
	string infile_2 = "/home/dayoff/codes/point_cloud_lib/data/kitti/0000000001.ply";

	// read first point cloud
	readPointCloud(cloud_1, infile_1);

	// read second point cloud
	readPointCloud(cloud_2, infile_2);

	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
		PointCloud<num_t>,
		3 /* dim */
		> my_kd_tree_t;

	my_kd_tree_t index_pc2(3 /*dim*/, cloud_2, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index_pc2.buildIndex();

	// ----------------------------------------------------------------
	// radiusSearch(): Perform a search for the points within search_radius
	// ----------------------------------------------------------------
	vector < int > intersec_idx, innov_point_cloud;
	{
		float THREASHOLD = 0.3;
		const num_t search_radius = static_cast<num_t>(THREASHOLD);
		vector<pair<size_t,num_t> > ret_matches;

		nanoflann::SearchParams params;
		// params.sorted = false;

		for (size_t i = 0; i < cloud_1.kdtree_get_point_count(); ++i) {
			const num_t query_pt[3] = {cloud_1.kdtree_get_pt(i, 0),
									   cloud_1.kdtree_get_pt(i, 1),
									   	cloud_1.kdtree_get_pt(i, 2)};
			const size_t nMatches = index_pc2.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

			cout << "p = " << query_pt[0] << ", " << query_pt[1] << ", " << query_pt[2] << endl;
			cout << "radius = " << search_radius << " -> " << nMatches << " matches"<< endl;
			if (nMatches > 0) {
				cout << "First -> idx[0] = " << ret_matches[0].first << " dist[0] = " << ret_matches[0].second << endl;
				for (size_t i = 0; i < nMatches; i++) {
					if (ret_matches[i].second <= THREASHOLD) {
						intersec_idx.push_back(ret_matches[i].first);
					}
				}
			}
		}

		cout << "Finished calculating the euclidean distance of all points to all points." << endl;
		cout << "Total points : " << intersec_idx.size() << endl;
		sort( intersec_idx.begin(), intersec_idx.end() );
		intersec_idx.erase( unique( intersec_idx.begin(), intersec_idx.end() ), intersec_idx.end() );
		cout << "Total unique points : " << intersec_idx.size() << endl;
		innov_point_cloud = getPointsInnov(cloud_2, intersec_idx);
	}
	
	string outfile = "/home/dayoff/codes/point_cloud_lib/data/innovations/0_1.ply";
	cout << "Saving output file to " << outfile << endl;
	savePointCloud(cloud_2, innov_point_cloud, outfile);
	cout << "Program finished." << endl;
}

int main() {
	kdtree_search<double>();
	return 0;
}