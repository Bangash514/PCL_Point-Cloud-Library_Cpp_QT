// BEN 2026-11-02
// Change aim: point-to-point ICP registration of two point clouds with PCL, with error against a known transform.
// File: icp_registration.cpp
// SIMULATED data only. Usage: icp_registration --data data --out results --tag cpp
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

static const double kMaxDist = 0.5;
static const int kMaxIter = 100;

static std::string getArg(int argc, char** argv, const std::string& name, const std::string& def)
{
    for (int i = 1; i + 1 < argc; ++i)
        if (name == argv[i]) return argv[i + 1];
    return def;
}

static Eigen::Matrix4d readMatrix(const std::string& path)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    std::ifstream in(path);
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) in >> m(i, j);
    return m;
}

static double rmseNN(const Cloud& src, pcl::KdTreeFLANN<pcl::PointXYZ>& tree, const Eigen::Matrix4d& m)
{
    double s = 0;
    for (size_t i = 0; i < src.size(); ++i) {
        const Eigen::Vector3d p = m.block<3, 3>(0, 0) * Eigen::Vector3d(src[i].x, src[i].y, src[i].z) + m.block<3, 1>(0, 3);
        pcl::PointXYZ q(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        std::vector<int> idx(1);
        std::vector<float> d2(1);
        tree.nearestKSearch(q, 1, idx, d2);
        s += d2[0];
    }
    return std::sqrt(s / static_cast<double>(src.size()));
}

int main(int argc, char** argv)
{
    const std::string dir = getArg(argc, argv, "--data", "data");
    const std::string outDir = getArg(argc, argv, "--out", "results");
    const std::string tag = getArg(argc, argv, "--tag", "cpp");

    Cloud::Ptr src(new Cloud), tgt(new Cloud);
    if (pcl::io::loadPLYFile(dir + "/source.ply", *src) < 0 || pcl::io::loadPLYFile(dir + "/target.ply", *tgt) < 0) {
        std::cerr << "cannot read PLY files in " << dir << "\n";
        return 1;
    }
    const Eigen::Matrix4d gt = readMatrix(dir + "/gt.txt");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaxCorrespondenceDistance(kMaxDist);
    icp.setMaximumIterations(kMaxIter);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);
    Cloud aligned;
    icp.align(aligned);
    const Eigen::Matrix4d m = icp.getFinalTransformation().cast<double>();

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(tgt);
    const double before = rmseNN(*src, tree, Eigen::Matrix4d::Identity());
    const double after = rmseNN(*src, tree, m);
    const double c = std::max(-1.0, std::min(1.0, ((m.block<3, 3>(0, 0).transpose() * gt.block<3, 3>(0, 0)).trace() - 1.0) / 2.0));
    const double rotErr = std::acos(c) * 180.0 / M_PI;
    const double transErr = (m.block<3, 1>(0, 3) - gt.block<3, 1>(0, 3)).norm();

    std::ostringstream o;
    o << std::fixed;
    o << "points: source " << src->size() << ", target " << tgt->size() << "\n";
    o << std::setprecision(4) << "nearest-neighbour RMSE before: " << before << ", after: " << after << "\n";
    o << std::setprecision(3) << "rotation error: " << rotErr << " deg, translation error: " << std::setprecision(4) << transErr << "\n";
    std::cout << o.str();
    std::ofstream(outDir + "/summary_" + tag + ".txt") << o.str();

    std::ofstream t(outDir + "/transform_" + tag + ".txt");
    t << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; ++i) t << m(i, 0) << " " << m(i, 1) << " " << m(i, 2) << " " << m(i, 3) << "\n";
    std::ofstream x(outDir + "/aligned_" + tag + ".xyz");
    x << std::fixed << std::setprecision(5);
    for (size_t i = 0; i < aligned.size(); ++i) x << aligned[i].x << " " << aligned[i].y << " " << aligned[i].z << "\n";
    return 0;
}
