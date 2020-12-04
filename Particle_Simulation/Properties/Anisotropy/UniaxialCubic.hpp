#pragma once

#ifndef INC_STOPHYS_UNIAXIALCUBICANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_UNIAXIALCUBICANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"
#include "Uniaxial.hpp"
#include "Cubic.hpp"

#include "General/MathTypes.hpp"

#include <random>

namespace Properties::Anisotropy
{
    template<typename prec>
    struct UniaxialCubic_Distribution;

    template<typename prec>
    struct UniaxialCubic : General<prec> {
        using ThisClass = UniaxialCubic<prec>;
        using Distribution = UniaxialCubic_Distribution<prec>;
        using Vector3D = SPhys::math::Vector3D<prec>;

        Uniaxial<prec>  uniaxial;
        Cubic<prec>     cubic;
        Vector3D        cubic_orientation { Vector3D::Zero()};

        // TODO: Add calculation of mean orientation. 
        ThisClass& operator+=(const ThisClass& rhs)
        {
            uniaxial    += rhs.uniaxial;
            cubic       += rhs.cubic;
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            uniaxial    /= scalar;
            cubic       /= scalar;
            return *this;
        }
    };
    template<typename Precision, typename Archive>
    void serialize(UniaxialCubic<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Uniaxial_Part", val.uniaxial));
        ar(Archives::createNamedValue("Cubic_Part", val.cubic));
        ar(Archives::createNamedValue("Cubic_Orientation", val.cubic_orientation));
    }

    // TODO
    template<typename prec>
    struct UniaxialCubic_Distribution : Distribution<prec>
    {
        using ThisClass = UniaxialCubic_Distribution<prec>;
        using Anisotropy = UniaxialCubic<prec>;

            Uniaxial_Distribution<prec> uniaxial_distribution;
            Cubic_Distribution<prec>    cubic_distribution;
            bool useRandomCubicOrientation {false};

        Anisotropy& applyDistribution(Anisotropy& val) 
        {
            val.uniaxial    = uniaxial_distribution.applyDistribution(val.uniaxial);
            val.cubic       = cubic_distribution.applyDistribution(val.cubic);
            if(useRandomCubicOrientation)
            {
                static thread_local std::uniform_real_distribution<prec> ud{ 0.0,1.0 };
                static thread_local auto prng{ math::random_helpers::create_seeded_PRNG<std::mt19937_64>(std::random_device{}) };
                val.cubic_orientation(0) = ud(prng) * math::constants::two_pi<prec>;
                val.cubic_orientation(1) = ud(prng) * math::constants::pi<prec>;
                val.cubic_orientation(2) = ud(prng) * math::constants::two_pi<prec>;
            }
            
            return val;
        }
     };

    template<typename Precision, typename Archive>
    void serialize(UniaxialCubic_Distribution<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Uniaxial_Distribution", val.uniaxial_distribution));
        ar(Archives::createNamedValue("Cubic_Distribution", val.cubic_distribution));
        ar(Archives::createNamedValue("UseRandomCubicOrientation", val.useRandomCubicOrientation));
    }
}
#endif //INC_STOPHYS_UNIAXIALCUBICANISOTROPYPROPERTY_HPP
