///-------------------------------------------------------------------------------------------------
// file:	VoxelInformation.h
//
// summary:	Declares the voxel information class
///-------------------------------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>

#include "Archive/NamedValue.h"

class VoxelInformation
{
	using VoxelPosition = Eigen::Matrix<std::size_t,3,1>; 

private:
	VoxelPosition mVoxel;
public:
	VoxelInformation(const VoxelPosition& voxel) : mVoxel(voxel) {};

	static std::string getSectionName() { return std::string{ "Voxel_Information" }; };

	template<typename Archive>
	void serialize(Archive &ar)
	{
		ar(Archives::createNamedValue("Voxel", mVoxel));
	}
};