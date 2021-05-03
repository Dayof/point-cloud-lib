#include "nanoflann.hpp"
#include "pc_utils.hpp"

#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <ctime>	

using namespace std;
using namespace nanoflann;

string DATA_PATH = "../../../data/global/";
string INNOV_PATH = "../../../data/innovation/";
string REF_PATH = "../../../data/reference/";
float RADIUS_THRESHOLD = 0.2;
int NZEROS = 10;
int PC_SIZE = 107;


PointCloud readNextPC(int index) {
	PointCloud cloud;

	cout << "Reading point cloud: " << index << endl;
	string idx_str = to_string(index);
	string infile = DATA_PATH + string(NZEROS - idx_str.length(), '0').append(idx_str).append(".ply");

	// read point cloud
	readPointCloud(cloud, infile);

	return cloud;
}


PointCloud kdtreeSearch(PointCloud &cloud_1, PointCloud &cloud_2) {
	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<float, PointCloud> ,
		PointCloud,
		3 /* dimension */
		> my_kd_tree_t;

	my_kd_tree_t index_pc_1(3 /* dimension */, cloud_1, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index_pc_1.buildIndex();

	// ----------------------------------------------------------------
	// radiusSearch(): Perform a search for the points within search_radius
	// ----------------------------------------------------------------
	vector <int> intersec_idx (cloud_2.kdtree_get_point_count(), 0);
	PointCloud pc_innov;
	{
		const float search_radius = static_cast<float>(RADIUS_THRESHOLD);
		vector<pair<size_t, float> > ret_matches;
		int intersec_counter = 0;

		nanoflann::SearchParams params;
		// params.sorted = false;

		for (size_t i = 0; i < cloud_2.kdtree_get_point_count(); ++i) {
			const float query_pt[3] = {cloud_2.kdtree_get_pt(i, 0),
									   cloud_2.kdtree_get_pt(i, 1),
									   	cloud_2.kdtree_get_pt(i, 2)};
			const size_t n_matches = index_pc_1.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

			// cout << "p = " << query_pt[0] << ", " << query_pt[1] << ", " << query_pt[2] << endl;
			// cout << "radius = " << search_radius << " -> " << n_matches << " matches"<< endl;
			if (n_matches == 0) {
				// cout << "No matches found for point p" << endl;
				intersec_idx[i] = 1;
				intersec_counter++;
			} else {
				// cout << "First -> idx[0] = " << ret_matches[0].first << " dist[0] = " << ret_matches[0].second << endl;
				// for (size_t i = 0; i < n_matches; i++) {
				// 	if (ret_matches[i].second <= RADIUS_THRESHOLD) {
				// 		intersec_idx[ret_matches[i].first] = 1;
				// 	}
				// }
			}
		}

		cout << "Finished calculating the distance of all points to all points." << endl;
		pc_innov = fromIdxToPointCloud(cloud_2, intersec_idx);
		cout << "Innovation point cloud with " << pc_innov.kdtree_get_point_count() << " points." << endl;
	}
	return pc_innov;
}


void savePointCloudCtrl(int idx, PointCloud &pc, string path) {
	string idx_str = to_string(idx);
	string outfile = path + string(NZEROS - idx_str.length(), '0').append(idx_str).append(".ply");
	cout << "Saving output file to " << outfile << endl;
	savePointCloud(pc, outfile);
}


PointCloud pcl_to_pc(pcl::PointCloud< pcl::PointXYZ > pc_pcl) {
	PointCloud cloud;
	cloud.pts.reserve(pc_pcl.size());
	for (auto point: pc_pcl) {
		PointCloud::Point pts_vec;
		pts_vec.x = point.x;
		pts_vec.y = point.y;
		pts_vec.z = point.z;
		cloud.pts.push_back(pts_vec);
	}
	return cloud;
}


pcl::PointCloud< pcl::PointXYZ > pc_to_pcl(PointCloud pc) {
	pcl::PointCloud< pcl::PointXYZ > cloud;
	cloud.points.resize(pc.pts.size());

	for (int i = 0; i < cloud.size(); ++i){
		cloud.points[i].x = pc.pts[i].x;
		cloud.points[i].y = pc.pts[i].y;
		cloud.points[i].z = pc.pts[i].z;
	}

	return cloud;
}


PointCloud makeRef(PointCloud pc_ref, PointCloud pc_cur) {
	pcl::PointCloud< pcl::PointXYZ > pcl_pc_ref, pcl_pc_cur, pcl_new_ref;
	PointCloud new_ref;
	
	cout << "REF: tranforming pc_ref to pcl" << endl;
	pcl_pc_ref = pc_to_pcl(pc_ref);
	cout << "REF: tranforming pc_cur to pcl" << endl;
	pcl_pc_cur = pc_to_pcl(pc_cur);

	pcl_new_ref = pcl_pc_ref;
	pcl_new_ref += pcl_pc_cur;

	new_ref = pcl_to_pc(pcl_new_ref);

	return new_ref;
}


int main() {
	PointCloud old_pc_ref, pc_ref, pc_cur, pc_innov;
	// PCQueue pc_ref;
	// pc_ref.push(n, pc_cur);

	int n = 0;
	cout << endl << "Reading PC: " << n << endl;
	pc_cur = readNextPC(n);

	cout << endl << "GENERATING PC REFERENCE: " << n << endl;
	pc_ref = makeRef(pc_ref, pc_cur);
	cout << endl << "SAVING PC REFERENCE: " << n << endl;
	savePointCloudCtrl(n, pc_ref, REF_PATH);
	cout << endl << "PCs Metadata: " << endl;
	cout << "pc_ref size: " <<  pc_ref.pts.size() << endl;
	cout << "pc_cur size: " <<  pc_cur.pts.size() << endl;
	cout << "pc_ref size: " <<  pc_ref.pts.size() << endl;

	for (n = 1; n <= PC_SIZE; ++n) {
		cout << endl << "Reading PC: " << n << endl;
		pc_cur = readNextPC(n);

		cout << endl << "GENERATING INNOVATION: " << n - 1 << endl;
		pc_innov = kdtreeSearch(pc_ref, pc_cur);
		cout << endl << "PCs Metadata: " << endl;
		cout << "pc_ref size: " <<  pc_ref.pts.size() << endl;
		cout << "pc_cur size: " <<  pc_cur.pts.size() << endl;
		cout << "pc_innov size: " <<  pc_innov.pts.size() << endl;
		cout << endl << "SAVING PC INNOVATION: " << n - 1 << endl;
		savePointCloudCtrl(n - 1, pc_innov, INNOV_PATH);

		cout << endl << "GENERATING PC REFERENCE: " << n << endl;
		old_pc_ref = pc_ref;
		pc_ref = makeRef(old_pc_ref, pc_innov);
		cout << endl << "SAVING PC REFERENCE: " << n << endl;
		savePointCloudCtrl(n, pc_ref, REF_PATH);
		cout << endl << "PCs Metadata: " << endl;
		cout << "old_pc_ref size: " <<  old_pc_ref.pts.size() << endl;
		cout << "pc_innov size: " <<  pc_innov.pts.size() << endl;
		cout << "new pc_ref size: " <<  pc_ref.pts.size() << endl;

		// pc_ref = makeICP(pc_ref, pc_innov);
		// pc_ref.push(n, pc_innov);
	}

	cout << "Program finished." << endl;

	return 0;
}
