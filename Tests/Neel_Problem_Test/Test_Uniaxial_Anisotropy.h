///---------------------------------------------------------------------------------------------------
// file:		Test_Uniaxial_Anisotropy.h
//
// summary: 	Declares the test uniaxial anisotropy class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Neumann
// date: 24.07.2017

#ifndef INC_Test_Uniaxial_Anisotropy_H
#define INC_Test_Uniaxial_Anisotropy_H
///---------------------------------------------------------------------------------------------------
#pragma once

//Google Test
#include <gtest/gtest.h>

//Properties Includes
#include "Properties/ParticleProperties.h"

//Selector Includes
#include "Selectors/AnisotropySelector.h"

//Anisotropy Include
#include "Problems/Anisotropy/UniaxialAnisotropy.h"


namespace
{
    class UniaxialAnisotropyTest : public ::testing::Test
    {
    public:
        static constexpr const ::Properties::IAnisotropy aniso = ::Properties::IAnisotropy::Anisotropy_uniaxial;

        using Precision = double;
        using Anisotropy = typename ::Selectors::AnisotropyTypeSelector<aniso>::type<Precision>;
        using TestProperties = typename ::Properties::MagneticProperties<Precision>;
        using Vec3D = Eigen::Matrix<Precision, 3, 1>;

        inline static auto createMagneticProperties()
        {
            //Magnetic Parameters
            const Precision damping = 0.1;
            const Precision gyro = 1.76E+11;
            const Precision Ms = 477464;
            const Precision rmag = 10E-9;
            const Precision KUni = 1E4;

            return TestProperties{ rmag,Ms,damping,gyro,aniso,::Properties::Anisotropy::Uniaxial<Precision>{ {}, KUni } };
        };

        TestProperties Properties;

        UniaxialAnisotropyTest() : Properties(createMagneticProperties()){};
    };


}



#endif	// INC_Test_Uniaxial_Anisotropy_H
// end of Test_Uniaxial_Anisotropy.h
///---------------------------------------------------------------------------------------------------

