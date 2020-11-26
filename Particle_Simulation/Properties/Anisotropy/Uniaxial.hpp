#pragma once

#ifndef INC_STOPHYS_UNIAXIALANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_UNIAXIALANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Uniaxial_Distribution;

    template<typename prec>
    struct Uniaxial : General<prec> {
        prec K_uniaxial{0.0};

        using ThisClass = Uniaxial<prec>;
        using Distribution = Uniaxial_Distribution<prec>;

        ThisClass& operator+=(const ThisClass& rhs)
        {
            K_uniaxial += rhs.K_uniaxial;
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            K_uniaxial /= (double)scalar;
            return *this;
        }
    };


    template<typename Precision, typename Archive>
    void serialize(Uniaxial<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("K_uniaxial", val.K_uniaxial));
    }

    template<typename prec>
    struct Uniaxial_Distribution : Distribution<prec>
    {
        bool useRelativeDistributionWidth {true};
        ::Distribution::IDistribution TypeOfDistribution {::Distribution::IDistribution::Distribution_normal };
        prec sigma_K_uniaxial {0.0};
        Uniaxial<prec>& applyDistribution(Uniaxial<prec>& val) 
        {
            if(!distribution)
            {
                if(useRelativeDistributionWidth)
                    init(val.K_uniaxial)
                else
                    init(1.0);
            }
            const auto dist = distribution->getValueFromDistribution();
            if (!useRelativeDistributionWidth)
                val.K_uniaxial = dist;
            else
                val.K_uniaxial *= dist;
            return val;
        }
    private:
        std::unique_ptr<::Distribution::IDistributionHelper<prec>> distribution;
        void init(const prec mean) {
            distribution = initDistribution(TypeOfDistribution, mean, sigma_K_uniaxial);
        }
    };

    template<typename Precision, typename Archive>
    void save(Uniaxial_Distribution<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Use_relative_distribution_width", val.useRelativeDistributionWidth));
        ar(Archives::createNamedValue("Sigma_K_uniaxial", val.sigma_K_uniaxial));
        ar(Archives::createNamedValue("Distribution_type", to_string(val.TypeOfDistribution)));
    }

    template<typename Precision, typename Archive>
    void load(Uniaxial_Distribution<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Use_relative_distribution_width", val.useRelativeDistributionWidth));
        ar(Archives::createNamedValue("Sigma_K_uniaxial", val.sigma_K_uniaxial));
        std::string tmp;
        ar(Archives::createNamedValue("Distribution_type", tmp));
        val.TypeOfDistribution = Distribution::from_string<Distribution::IDistribution>(tmp);
    }

}
#endif //INC_STOPHYS_UNIAXIALANISOTROPYPROPERTY_HPP
