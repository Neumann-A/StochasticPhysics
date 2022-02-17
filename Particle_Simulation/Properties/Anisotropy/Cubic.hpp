#pragma once

#ifndef INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP
#define INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP

#include "Anisotropy.hpp"

namespace Properties::Anisotropy
{
    template<typename prec>
    struct Cubic_Distribution;

    template<typename prec>
    struct Cubic : General<prec> {
        prec K_cubic{0.0};

        using ThisClass = Cubic<prec>;
        using Distribution = Cubic_Distribution<prec>;

        ThisClass& operator+=(const ThisClass& rhs)
        {
            K_cubic += rhs.K_cubic;
            return *this;
        }
        template<typename Number>
        std::enable_if_t<std::is_arithmetic<Number>::value, ThisClass&> operator/=(const Number& scalar)
        {
            K_cubic /= (double)scalar;
            return *this;
        }
    };


    template<typename Precision, typename Archive>
    void serialize(Cubic<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("K_cubic", val.K_cubic));
    }

    // TODO
    template<typename prec>
    struct Cubic_Distribution : Distribution<prec>
    {
        using ThisClass = Cubic_Distribution<prec>;
        using Anisotropy = Cubic<prec>;
        bool useRelativeDistributionWidth {true};
        ::Distribution::IDistribution TypeOfDistribution {::Distribution::IDistribution::Distribution_normal };
        prec sigma_K_cubic {0.0};
        Anisotropy& applyDistribution(Anisotropy& val)
        {
            if(!distribution)
            {
                if (!useRelativeDistributionWidth)
                    init(val.K_cubic);
                else
                    init(1.0);
            }
            const auto dist = distribution->getValueFromDistribution();
            if (!useRelativeDistributionWidth)
                val.K_cubic = dist;
            else
                val.K_cubic *= dist;
            return val;
        }
        Cubic_Distribution() = default;
        Cubic_Distribution(const ThisClass& other)
            : useRelativeDistributionWidth(other.useRelativeDistributionWidth),
              TypeOfDistribution(other.TypeOfDistribution),
              sigma_K_cubic(other.sigma_K_cubic) {}
        Cubic_Distribution& operator=(ThisClass other) { std::swap(other,*this); return *this; }
    private:
        std::unique_ptr<::Distribution::IDistributionHelper<prec>> distribution {nullptr};
        void init(const prec mean) {
            distribution = initDistribution(TypeOfDistribution, mean, sigma_K_cubic);
        }
    };

    template<typename Precision, typename Archive>
    void save(const Cubic_Distribution<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Use_relative_distribution_width", val.useRelativeDistributionWidth));
        ar(Archives::createNamedValue("Sigma_K_cubic", val.sigma_K_cubic));
        ar(Archives::createNamedValue("Distribution_type", to_string(val.TypeOfDistribution)));
    }

    template<typename Precision, typename Archive>
    void load(Cubic_Distribution<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Use_relative_distribution_width", val.useRelativeDistributionWidth));
        ar(Archives::createNamedValue("Sigma_K_cubic", val.sigma_K_cubic));
        std::string tmp;
        ar(Archives::createNamedValue("Distribution_type", tmp));
        val.TypeOfDistribution = ::Distribution::from_string<::Distribution::IDistribution>(tmp);
    }
}

#endif //INC_STOPHYS_CUBICANISOTROPYPROPERTY_HPP
