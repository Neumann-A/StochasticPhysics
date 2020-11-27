#pragma once

#ifndef INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Mixed_Distribution;

    template<typename prec>
    struct Mixed : General<prec> {

        using ThisClass = Mixed<prec>;
        using Distribution = Mixed_Distribution<prec>;

        ThisClass& operator+=(const ThisClass& rhs)
        {
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            return *this;
        }
    };
    template<typename Precision, typename Archive>
    void serialize(Mixed<Precision>& val, Archive& ar)
    {

    }

    // TODO
    template<typename prec>
    struct Mixed_Distribution : Distribution<prec>
    {
        using ThisClass = Mixed_Distribution<prec>;
        using Anisotropy = Mixed<prec>;
        bool useRelativeDistributionWidth {true};
        ::Distribution::IDistribution TypeOfDistribution {::Distribution::IDistribution::Distribution_normal };
        prec sigma_K_uniaxial {0.0};
        Anisotropy& applyDistribution(Anisotropy& val) 
        {
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
            // distribution = initDistribution(TypeOfDistribution, mean, sigma_K_uniaxial);
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
#endif //INC_STOPHYS_MIXEDANISOTROPYPROPERTY_HPP
