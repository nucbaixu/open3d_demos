#include "open3d/Open3D.h"

using namespace open3d;

void PrintVoxelGridInfomation(const geometry::VoxelGrid & voxel_grid){
	utility::LogInfo("geometry::voxelGrid width {:d} voxel", voxel_grid.voxels_.size());
	utility::LogInfo("origin:[{:f} {:f} {:f}]", voxel_grid.origin_(0),
		voxel_grid.origin_(1),
		voxel_grid.origin_(2));

	utility::LogInfo("               voxel_size: {:f}", voxel_grid.voxel_size_);
	return;
}

void PrintHelp() 
{
    PrintOpen3DVersion();
    // clang-format off
    utility::LogInfo("Usage:");
    utility::LogInfo("    > Voxelization [pointcloud_filename] [voxel_filename_ply]");
    // clang-format on
    utility::LogInfo("");


}
int main(int argc, char* argv[])
{

    argc = 3;
    argv[1] = "../data/1.pcd";
    argv[2] = "../data/1_out_voxelgrid.pcd";


	utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if (argc != 3 ||
        utility::ProgramOptionExistsAny(argc, argv, { "-h", "--help" })) {
        PrintHelp();
        return 1;
    }

    auto pcd = io::CreatePointCloudFromFile(argv[1]);
    auto voxel = geometry::VoxelGrid::CreateFromPointCloud(*pcd, 0.005);
    PrintVoxelGridInfomation(*voxel);
    visualization::DrawGeometries({pcd,voxel});
    io::WriteVoxelGrid(argv[2],*voxel,true);

    auto voxel_read = io::CreateVoxelGridFromFile(argv[2]);
    PrintVoxelGridInfomation(*voxel_read);
    visualization::DrawGeometries({pcd,voxel_read});

    return 0;
}