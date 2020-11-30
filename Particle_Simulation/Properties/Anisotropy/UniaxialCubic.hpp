#pragma once

#ifndef INC_STOPHYS_UNIAXIALCUBICANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_UNIAXIALCUBICANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"
#include "Uniaxial.hpp"
#include "Cubic.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct UniaxialCubic_Distribution;

    template<typename prec>
    struct UniaxialCubic : General<prec> {
        using ThisClass = UniaxialCubic<prec>;
        using Distribution = UniaxialCubic_Distribution<prec>;

        Uniaxial<prec>  uniaxial;
        Cubic<prec>     cubic;

        ThisClass& operator+=(const ThisClass& rhs)
        {
            uniaxial    += rhs.uniaxial;
            cubic       += rhs.cubic;
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            uniaxial    /= rhs.uniaxial;
            cubic       /= rhs.cubic;
            return *this;
        }
    };
    template<typename Precision, typename Archive>
    void serialize(UniaxialCubic<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Uniaxial_Part", uniaxial));
        ar(Archives::createNamedValue("Cubic_Part", cubic));
    }

    // TODO
    template<typename prec>
    struct UniaxialCubic_Distribution : Distribution<prec>
    {
        using ThisClass = UniaxialCubic_Distribution<prec>;
        using Anisotropy = UniaxialCubic<prec>;

            Uniaxial_Distribution<prec> uniaxial_distribution;
            Cubic_Distribution<prec>    cubic_distribution;

        Anisotropy& applyDistribution(Anisotropy& val) 
        {
            val.uniaxial    = applyDistribution(val.uniaxial);
            val.cubic       = applyDistribution(val.cubic);
            return val;
            // if(!distribution)
            // {
            //     if(useRelativeDistributionWidth)
            //         init(val.K_uniaxial)
            //     else
            //         init(1.0);
            // }
            // const auto dist = distribution->getValueFromDistribution();
            // if (!useRelativeDistributionWidth)
            //     val.K_uniaxial = dist;
            // else
            //     val.K_uniaxial *= dist;
        }
    private:
        // std::unique_ptr<::Distribution::IDistributionHelper<prec>> distribution;
        void init(const prec mean) {

        }
    };

    template<typename Precision, typename Archive>
    void save(const Mixed_Distribution<Precision>& val, Archive& ar)
    {
        // ar(Archives::createNamedValue("Use_relative_distribution_width", val.useRelativeDistributionWidth));
        // ar(Archives::createNamedValue("Sigma_K_uniaxial", val.sigma_K_uniaxial));
        // ar(Archives::createNamedValue("Distribution_type", to_string(val.TypeOfDistribution)));
    }

    template<typename Precision, typename Archive>
    void load(Mixed_Distribution<Precision>& val, Archive& ar)
    {
        // ar(Archives::createNamedValue("Use_relative_distribution_width", val.useRelativeDistributionWidth));
        // ar(Archives::createNamedValue("Sigma_K_uniaxial", val.sigma_K_uniaxial));
        // std::string tmp;
        // ar(Archives::createNamedValue("Distribution_type", tmp));
        // val.TypeOfDistribution = ::Distribution::from_string<::Distribution::IDistribution>(tmp);
    }

}
#endif //INC_STOPHYS_UNIAXIALCUBICANISOTROPYPROPERTY_HPP
