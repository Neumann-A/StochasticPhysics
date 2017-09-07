///---------------------------------------------------------------------------------------------------
// file:		Test_Neel_Spherical_Problem.h
//
// summary: 	Declares the test neel spherical problem class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 07.09.2017

#ifndef INC_Test_Neel_Spherical_Problem_H
#define INC_Test_Neel_Spherical_Problem_H
///---------------------------------------------------------------------------------------------------
#pragma once

//Eigen Core
#include <Eigen/Core>

//Google Test
#include <gtest/gtest.h>

//Properties Includes
#include "Properties/ParticleProperties.h"

//Problem Headers
#include "Selectors/ProblemSelector.h"
#include "Problems/Anisotropy/UniaxialAnisotropy.h"
#include "Problems/NeelRelaxationSpherical.h"

class NeelSphericalProblemTest : public ::testing::Test
{
	static constexpr const ::Properties::IAnisotropy sAni = ::Properties::IAnisotropy::Anisotropy_uniaxial;

public:
	using Precision = double;
	using Anisotropy = typename ::Selectors::AnisotropyTypeSelector<sAni>::type<Precision>;
	using Problem = typename ::Problems::NeelRelaxationSpherical<Precision, Anisotropy>;
	using Properties = typename Problem::UsedProperties;
	using InitSettings = typename Problem::InitSettings;
	using ProblemSettings = typename Problem::ProblemSettings;
	using Vec3D = Eigen::Matrix<Precision, 3, 1>;
	using Matrix3x3 = Eigen::Matrix<Precision, 3, 3>;
private:
	inline static ProblemSettings createProblemSettings()
	{
		return ProblemSettings{};
	}
	inline static InitSettings createInitializationSettings()
	{
		//constexpr const Precision pi{ 3.1415926535897932384626433832795 };

		Vec3D Pos, Orientation, MagDir;
		Pos << 0, 0, 0; //Unimportant
		Orientation << 0, 0, 0; //Euler Angles!; Defines Easy Axis Direction
		MagDir << 1, 0, 0; //Starting Direction of Magnetisation; Mainly unimportant for test;
		return InitSettings(false, false, false, Pos, Orientation, MagDir);
	}
	inline static Properties createProperties()
	{
		//General Parameters
		const Precision T = 295;
		const Precision visc = 1E-3;

		//Magnetic Parameters
		const Precision damping = 0.1;
		const Precision gyro = 1.76E+11;
		const Precision Ms = 4.77464E5;
		const Precision rmag = 10E-9;
		const Precision KUni = 1E4;

		//Hydrodynamic parameters
		const Precision rhydro = 100E-9;

		const ::Properties::MagneticProperties<Precision> MagProps{ rmag,Ms,damping,gyro,sAni,std::vector<Precision>{ {KUni,0.0} } };
		const ::Properties::HydrodynamicProperties<Precision> HydroProps{ rhydro,visc };

		return ::Properties::ParticlesProperties<Precision>{T, MagProps, HydroProps};
	}

public:
	ProblemSettings mSettings;
	InitSettings	mInitSet;
	Properties		mProperties;
	Problem			mProblem;

	inline NeelSphericalProblemTest()
		: mSettings(createProblemSettings()), mInitSet(createInitializationSettings()), mProperties(createProperties()),
		mProblem(mSettings, mProperties, mInitSet)
	{};

};
#endif	// INC_Test_Neel_Spherical_Problem_H
// end of Test_Neel_Spherical_Problem.h
