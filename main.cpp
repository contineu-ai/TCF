#include "registration/tcf.h"
#include "utils/for_cloud.h"
#include "utils/for_io.h"
#include "utils/for_time.h"
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
    
    // Reading command line args
    std::string path_source_cloud = argv[1];
    std::string path_target_cloud = argv[2];
    std::string path_matches = argv[3];
    std::string output_path = argv[4];
    // std::string path_source_cloud = config["path_source_cloud"];
    // std::string path_target_cloud = config["path_target_cloud"];
    // std::string path_matches = config["path_matches"];

    // Load point cloud and resolution
    // this can be replaced by a user-defined value
    CloudPtr source_cloud(new PointCloud), target_cloud(new PointCloud);
    pcl::io::loadPCDFile(path_source_cloud, *source_cloud);
    pcl::io::loadPCDFile(path_target_cloud, *target_cloud);
    float rs = pcResolution(source_cloud);
    float rt = pcResolution(target_cloud);
    float th = std::max(rs, rt);
    std::cout << "Resolution: " << th << " m\n";

    // Load correspondences
    Eigen::MatrixXf matches;
    loadMatrixDynamic(path_matches, matches);
    MatfD3 source_match = matches.leftCols(3);
    MatfD3 target_match = matches.rightCols(3);

    // Start registration
    TicToc tic_tcf;
    std::srand(unsigned(std::time(nullptr)));
    Eigen::Matrix4f trans = twoStageConsensusFilter(source_match, target_match, 3*th);
    double time_registration = tic_tcf.toc();
    std::cout << "Runtime: " << time_registration << " ms.\n";
    std::cout << "TCF pose: \n" << trans << "\n";

    std::ofstream out_file(output_path);
    if (!out_file.is_open()) {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
        return 1;
    }

    out_file << std::fixed << std::setprecision(15);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++) {
            out_file << trans(i, j) << " ";
        }
        out_file << std::endl;
    }

    // compute error
    // std::pair<double, double> error = computeTransError(trans, gt);
    // std::cout << "RE: " << error.first << " deg, TE: " << error.second << " m.\n";
    
    return 0;
}