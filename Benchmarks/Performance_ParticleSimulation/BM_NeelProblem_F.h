///---------------------------------------------------------------------------------------------------
// file:		BM_NeelProblem_F.h
//
// summary: 	Declares the bm neel problem class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Alexander
// date: 27.07.2017

#ifndef INC_BM_NeelProblem_F_H
#define INC_BM_NeelProblem_F_H
///---------------------------------------------------------------------------------------------------
#pragma once
#include "stdafx.h"

//Properties Includes
#include "Properties/ParticleProperties.h"

//Problem Headers
#include "Selectors/ProblemSelector.h"
#include "Problems/Anisotropy/UniaxialAnisotropy.h"
#include "Problems/NeelRelaxation.h"

#ifdef _MSC_VER
#pragma comment (lib, "shlwapi")
#endif

class BM_NeelProblem_F : public ::benchmark::Fixture
{
    static constexpr const ::Properties::IAnisotropy sAni = ::Properties::IAnisotropy::Anisotropy_uniaxial;

public:
    using Precision = double;
    using Anisotropy = typename ::Selectors::AnisotropyTypeSelector<sAni>::type<Precision>;
    using Problem = typename ::Problems::NeelRelaxation<Precision, Anisotropy>;
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

        const ::Properties::MagneticProperties<Precision> MagProps{ rmag,Ms,damping,gyro,{sAni, ::Properties::Anisotropy::Uniaxial<Precision>{ {}, KUni }} };
        const ::Properties::HydrodynamicProperties<Precision> HydroProps{ rhydro,visc };

        return ::Properties::ParticlesProperties<Precision>{T, MagProps, HydroProps};
    }

public:
    ProblemSettings mSettings;
    InitSettings	mInitSet;
    Properties		mProperties;
    Problem			mProblem;

    inline BM_NeelProblem_F()
        : mSettings(createProblemSettings()), mInitSet(createInitializationSettings()), mProperties(createProperties()),
        mProblem(mSettings, mProperties, mInitSet)
    {};
};



#endif	// INC_BM_NeelProblem_F_H
// end of BM_NeelProblem_F.h
///---------------------------------------------------------------------------------------------------
