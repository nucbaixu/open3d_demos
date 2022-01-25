#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "open3d/Open3D.h"

void printpointcloud(const open3d::geometry::PointCloud & cloud)
{
	using namespace open3d;

	bool pointcloud_has_noraml = cloud.HasNormals();
	utility::LogInfo("Point cloud has {:d} points",cloud.points_.size());

	Eigen::Vector3d min_bound = cloud.GetMinBound();
	Eigen::Vector3d max_bound = cloud.GetMaxBound();

	utility::LogInfo(
		"Bounding box is:({:.4f},{:.4f},{:.4f})-({:.4f},{:.4f},{:.4f})",
		min_bound(0), min_bound(1), min_bound(2),
		max_bound(0), max_bound(1), max_bound(2));

	for (size_t i = 0; i < cloud.points_.size(); i++)
	{
		if (pointcloud_has_noraml)
		{
			const Eigen::Vector3d& point = cloud.points_[i];
			const Eigen::Vector3d& normal = cloud.normals_[i];

			utility::LogInfo("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}",
				point(0), point(1), point(2), normal(0), normal(1),
				normal(2));
		}else
		{
			const Eigen::Vector3d& point = cloud.points_[i];
			utility::LogInfo("{:.6f} {:.6f} {:.6f}", point(0), point(1),
				point(2));
		}

		utility::LogInfo("End of the list.");
	}
}

void PrintHelp() {
	using namespace open3d;

	PrintOpen3DVersion();
	// clang-format off
	utility::LogInfo("Usage:");
	utility::LogInfo("    > PointCloud [pointcloud_filename]");
	// clang-format on
	utility::LogInfo("");
}

int main(int argc,char * argv[])
{
	using namespace open3d;

	utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

	if (argc != 2 ||
		utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
		PrintHelp();
		return 1;
	}

	auto pcd = io::CreatePointCloudFromFile(argv[1]);
	{
		utility::ScopeTimer timer("FPFH estimation with Radius 0.25");
		pipelines::registration::ComputeFPFHFeature(*pcd,open3d::geometry::KDTreeSearchParamRadius(0.25));
	}

	{
		utility::ScopeTimer timer("Normal estimation with KNN20");
		for (int i = 0; i < 20; i++) {
			pcd->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));
		}
	}

	std::cout << pcd->normals_[0] << std::endl;
	std::cout << pcd->normals_[10] << std::endl;

	{
		utility::ScopeTimer timer("Normal estimation with Radius 0.01666");
		for (int i = 0; i < 20; i++) {
			pcd->EstimateNormals(
				open3d::geometry::KDTreeSearchParamRadius(0.01666));
		}
	}
	std::cout << pcd->normals_[0] << std::endl;
	std::cout << pcd->normals_[10] << std::endl;

	{
		utility::ScopeTimer timer("Normal estimation with Hybrid 0.01666, 60");
		for (int i = 0; i < 20; i++) {
			pcd->EstimateNormals(
				open3d::geometry::KDTreeSearchParamHybrid(0.01666, 60));
		}
	}
	std::cout << pcd->normals_[0] << std::endl;
	std::cout << pcd->normals_[10] << std::endl;


	auto downpcd = pcd->VoxelDownSample(0.05);

	// 1. test basic pointcloud functions.

	geometry::PointCloud pointcloud;
	printpointcloud(pointcloud);

	pointcloud.points_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
	pointcloud.points_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
	pointcloud.points_.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
	pointcloud.points_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
	printpointcloud(pointcloud);

	visualization::Visualizer visualizer;
	std::shared_ptr<geometry::PointCloud> pointcloud_ptr(
		new geometry::PointCloud);
	*pointcloud_ptr = pointcloud;
	pointcloud_ptr->NormalizeNormals();
	auto bounding_box = pointcloud_ptr->GetAxisAlignedBoundingBox();

	//Eigen Block:https://blog.csdn.net/zhangqian_shai/article/details/101756145
	//Eigen Block:https://www.cnblogs.com/ymxiansen/p/5259570.html
	std::shared_ptr<geometry::PointCloud> pointcloud_transformed_ptr(
		new geometry::PointCloud);
	*pointcloud_transformed_ptr = *pointcloud_ptr;
	Eigen::Matrix4d trans_to_origin = Eigen::Matrix4d::Identity();
	trans_to_origin.block<3, 1>(0, 3) = bounding_box.GetCenter() * -1.0;
	Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
	transformation.block<3, 3>(0, 0) = static_cast<Eigen::Matrix3d>(
		Eigen::AngleAxisd(3.1415926 / 4.0, Eigen::Vector3d::UnitX()));
	pointcloud_transformed_ptr->Transform(trans_to_origin.inverse() *
		transformation * trans_to_origin);

	visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
	visualizer.AddGeometry(pointcloud_ptr);
	visualizer.AddGeometry(pointcloud_transformed_ptr);
	visualizer.Run();
	visualizer.DestroyVisualizerWindow();

	// 4. test operations
	*pointcloud_transformed_ptr += *pointcloud_ptr;
	visualization::DrawGeometries({ pointcloud_transformed_ptr },
		"Combined Pointcloud");

	// 5. test downsample
	auto downsampled = pointcloud_ptr->VoxelDownSample(0.05);
	visualization::DrawGeometries({ downsampled }, "Down Sampled Pointcloud");

	// 6. test normal estimation
	visualization::DrawGeometriesWithKeyCallbacks(
		{ pointcloud_ptr },
		{ {GLFW_KEY_SPACE,
		  [&](visualization::Visualizer* vis) {
			// EstimateNormals(*pointcloud_ptr,
			//        open3d::KDTreeSearchParamKNN(20));
			pointcloud_ptr->EstimateNormals(
					open3d::geometry::KDTreeSearchParamRadius(0.05));
			utility::LogInfo("Done.");
			return true;
		}} },
		"Press Space to Estimate Normal", 1600, 900);

	// n. test end

	utility::LogInfo("End of the test.");
	return 0;
}