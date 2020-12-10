/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

//#include "BrownAndNeelRelaxation.h" //File is included by Header!

//IMPORTANT: NEED TO RECHECK ALL SIGNS -> CREATE TEST CASES!

#include <random>

#ifdef INC_BROWNANDNEELRELAXATION_H_

namespace Problems
{
    template<typename precision, typename aniso, bool SimpleModel>
    BrownAndNeelRelaxation<precision, aniso, SimpleModel>::BrownAndNeelRelaxation(const ProblemSettings& ProbSettings, const UsedProperties &Properties, const InitSettings& Init) :
        GeneralSDEProblem<BrownAndNeelRelaxation<precision, aniso>>(BrownAndNeelDimensionVar),
        //toStochasticMatrix(ProbSettings.getUseSimpleModel() ? &BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixSimplified : &BrownAndNeelRelaxation<precision, aniso>::getStochasticMatrixFull),
        //toDrift(ProbSettings.getUseSimpleModel() ? &BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoSimplified : &BrownAndNeelRelaxation<precision, aniso>::getStratonovichtoItoFull),
        _ParamHelper(Properties),
        mNeelParams(Helpers::NeelCalculator<Precision>::calcNeelParams(Properties.getMagneticProperties(), Properties.getTemperature())),
        mBrownParams(Helpers::BrownianRotationCalculator<Precision>::calcBrownRotationParams(Properties.getHydrodynamicProperties(), Properties.getTemperature())), 
        MagneticMoment(Properties.getMagneticProperties().getSaturationMoment()),
        _Init(Init), mProblemSettings(ProbSettings),
        mAnisotropy(Properties.getMagneticProperties())
    {}


    //Get the stoachstig matrix
    template<typename precision, typename aniso, bool SimpleModel>
    BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStochasticMatrix(const DependentType& yi) const noexcept-> StochasticMatrixType
    {
        return detail::BrownStochasticMatrixSelector<SimpleModel>::SelectImpl(*this, yi);
        //return (this->*toStochasticMatrix)(yi);
    }

    //Actual Calculation of the Stochastic Matrix; Full model with noise coupling
    template<typename precision, typename aniso, bool SimpleModel>
    inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStochasticMatrixFull(const DependentType& yi) const noexcept -> StochasticMatrixType
    {
        const auto& ni{ (yi.template head<3>()) };// Brown Direction Vector 
        const auto& ei{ (yi.template tail<3>()) };// Neel Direction Vector  

        const auto& a = mNeelParams.NeelFactor1;
        const auto& b = mNeelParams.NeelFactor2;
        const auto& c = mBrownParams.BrownPrefactor;
        const auto d = c*MagneticMoment;

        StochasticMatrixType StochasticMatrix; // Return Matrix
        //Block Expresions
        auto Brown_F{ StochasticMatrix.template topLeftCorner<3, 3>() };
        auto Brown_H{ StochasticMatrix.template topRightCorner<3, 3>() };
        auto Neel_F{ StochasticMatrix.template bottomLeftCorner<3, 3>() };
        auto Neel_H{ StochasticMatrix.template bottomRightCorner<3, 3>() };

        //Brown_F_Noise = c*Drift
        const auto Brown_F_Noise = mBrownParams.Brown_F_Noise;
        const auto d_H_Noise = d*mNeelParams.NoisePrefactor;
        /* BEGIN Mixed Terms describing the coupling */
        {
            const auto dni = (d_H_Noise*ni).eval();
            Brown_H = Matrix3x3::Identity()*dni.dot(ei);
            Brown_H -= ei*dni.transpose();
        }
        {
            const auto cTmi = (Brown_F_Noise*ei).eval();
            Neel_F(0, 0) = 0.0;
            Neel_F(1, 0) = -cTmi(2);
            Neel_F(2, 0) = cTmi(1);
            Neel_F(0, 1) = cTmi(2);
            Neel_F(1, 1) = 0.0;
            Neel_F(2, 1) = -cTmi(0);
            Neel_F(0, 2) = -cTmi(1);
            Neel_F(1, 2) = cTmi(0);
            Neel_F(2, 2) = 0.0;
        }
        /* End Mixed Terms */

        /* BEGIN Brown Rotation */
        {
            const auto cTni = (Brown_F_Noise*ni).eval();
            Brown_F(0, 0) = 0.0;
            Brown_F(1, 0) = -cTni(2);
            Brown_F(2, 0) = cTni(1);
            Brown_F(0, 1) = cTni(2);
            Brown_F(1, 1) = 0.0;
            Brown_F(2, 1) = -cTni(0);
            Brown_F(0, 2) = -cTni(1);
            Brown_F(1, 2) = cTni(0);
            Brown_F(2, 2) = 0.0;
        }
        /* END Brown Rotation */

        /* BEGIN Neel Rotation*/
        {
            auto outerei = (Matrix3x3::Identity()-ei*ei.transpose()).eval();
            Neel_H = (b + d)*mNeelParams.NoisePrefactor*outerei;
            auto aei{ (a*mNeelParams.NoisePrefactor*ei).eval() };
            
            Neel_H(1, 0) += aei(2);
            Neel_H(2, 0) -= aei(1);
            Neel_H(0, 1) -= aei(2);
            Neel_H(2, 1) += aei(0);
            Neel_H(0, 2) += aei(1);
            Neel_H(1, 2) -= aei(0);
        }
        /* END Neel Rotation*/

        return StochasticMatrix;
    }

    //Simplified version; ignoring noise coupling between brown and neel
    template<typename precision, typename aniso, bool SimpleModel>
    inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStochasticMatrixSimplified(const DependentType& yi) const noexcept-> StochasticMatrixType
    {
        const auto& ni{ (yi.template head<3>()) };// Brown Direction Vector 
        const auto& ei{ (yi.template tail<3>()) };// Neel Direction Vector  

        const auto& a = mNeelParams.NeelFactor1;
        const auto& b = mNeelParams.NeelFactor2;
        //const auto& c = mBrownParams.BrownPrefactor;
        //const auto d = c*MagneticMoment;

        StochasticMatrixType StochasticMatrix; // Return Matrix
                                               //Block Expresions
        auto Brown_F{ StochasticMatrix.template topLeftCorner<3, 3>() };
        auto Brown_H{ StochasticMatrix.template topRightCorner<3, 3>() };
        auto Neel_F{ StochasticMatrix.template bottomLeftCorner<3, 3>() };
        auto Neel_H{ StochasticMatrix.template bottomRightCorner<3, 3>() };

        //Brown_F_Noise = c*Drift
        const auto Brown_F_Noise = mBrownParams.Brown_F_Noise;
        /* BEGIN Mixed Terms describing the coupling */
        Brown_H = Matrix3x3::Zero();
        Neel_F = Matrix3x3::Zero();
        /* End Mixed Terms */

        /* BEGIN Brown Rotation */
        {
            const auto cTni = (Brown_F_Noise*ni).eval();
            Brown_F(0, 0) = 0.0;
            Brown_F(1, 0) = -cTni(2);
            Brown_F(2, 0) = cTni(1);
            Brown_F(0, 1) = cTni(2);
            Brown_F(1, 1) = 0.0;
            Brown_F(2, 1) = -cTni(0);
            Brown_F(0, 2) = -cTni(1);
            Brown_F(1, 2) = cTni(0);
            Brown_F(2, 2) = 0.0;
        }
        /* END Brown Rotation */

        /* BEGIN Neel Rotation*/
        {
            auto outerei = (Matrix3x3::Identity() - ei*ei.transpose()).eval();
            Neel_H = (b)*mNeelParams.NoisePrefactor*outerei;
            auto aei{ (a*mNeelParams.NoisePrefactor*ei).eval() };

            Neel_H(1, 0) += aei(2);
            Neel_H(2, 0) -= aei(1);
            Neel_H(0, 1) -= aei(2);
            Neel_H(2, 1) += aei(0);
            Neel_H(0, 2) += aei(1);
            Neel_H(1, 2) -= aei(0);
        }
        /* END Neel Rotation*/

        return StochasticMatrix;
    }

    //Gets the Drift term
    template<typename precision, typename aniso, bool SimpleModel>
    BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getDrift(const DependentType& yi) const noexcept-> DeterministicType
    {
        return detail::BrownDriftSelector<SimpleModel>::SelectImpl(*this, yi);
    }

    template<typename precision, typename aniso, bool SimpleModel>
    inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStratonovichtoItoSimplified(const DependentType& yi) const noexcept-> DeterministicType
    {
        const auto& ni{ yi.template head<3>() };// Brown Direction Vector 
        const auto& ei{ yi.template tail<3>() };// Neel Direction Vector  

        const auto& a = mNeelParams.NeelFactor1;
        const auto& b = mNeelParams.NeelFactor2;
        //const auto& c = mBrownParams.BrownPrefactor;
        //Brown_F_Noise = c*Drift
        const auto Brown_F_Noise = mBrownParams.Brown_F_Noise;
        const auto cF_2 = Brown_F_Noise*Brown_F_Noise;
        const auto PreH_2 = mNeelParams.NoisePrefactor*mNeelParams.NoisePrefactor;
        DeterministicType result;
        result.template head<3>().noalias() = -ni*(cF_2);
        result.template tail<3>().noalias() = -ei*((a*a + b*b)*PreH_2);
        return result;
    }

    template<typename precision, typename aniso, bool SimpleModel>
    inline auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStratonovichtoItoFull(const DependentType& yi) const noexcept -> DeterministicType
    {
        const auto& ni{ yi.template head<3>() };// Brown Direction Vector 
        const auto& ei{ yi.template tail<3>() };// Neel Direction Vector  

        const auto& a = mNeelParams.NeelFactor1;
        const auto& b = mNeelParams.NeelFactor2;
        const auto& c = mBrownParams.BrownPrefactor;
        const auto d = c*MagneticMoment;
        //Brown_F_Noise = c*Drift
        const auto Brown_F_Noise = mBrownParams.Brown_F_Noise;
        const auto cF_2 = Brown_F_Noise*Brown_F_Noise;
        const auto PreH_2 = mNeelParams.NoisePrefactor*mNeelParams.NoisePrefactor;
        const auto bpd = b + d;
        //const auto nixei = ni.cross(ei);

        DeterministicType result;
        result.template head<3>().noalias() = -ni*(cF_2)+(d*a*ni.cross(ei)-0.5*d*d*((ni*ei.transpose())*ei-ni))*PreH_2;
        result.template tail<3>().noalias() = -ei*(cF_2+(a*a+ bpd*bpd)*PreH_2);
        return result;
    }

    //Actual Calculation of the Deterministic Matrix (no approx needed) (no difference between simple and full model)
    template<typename precision, typename aniso, bool SimpleModel>
    BASIC_ALWAYS_INLINE auto BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getDeterministicVector(const DependentType& yi, const IndependentType& xi) const noexcept-> DeterministicType
    {


        if constexpr (aniso::traits::value == aniso::traits::value_type::Anisotropy_cubic)
        {
            //TODO: Implement cubic in this case!
            return DeterministicType::Zero();
        }
        else
        {
            //Faster than any 4D Version
            const auto& ni{ yi.template head<3>() }; // Brown Direction Vector 
            const auto& ei{ yi.template tail<3>() }; // Neel Direction Vector  

            const auto xAxis = IndependentType::Zero();
            const auto yAxis = IndependentType::Zero();
            const auto& zAxis = ni;

            mAnisotropy.prepareField(ei, xAxis, yAxis, zAxis);
            const auto Heff{ (mAnisotropy.getAnisotropyField(ei,xAxis,yAxis,zAxis) + xi) };
            const auto Teff{ (mAnisotropy.getEffTorque(ei,xAxis,yAxis,zAxis,IndependentType::Zero(),IndependentType::Zero(),IndependentType::Zero())) };
            //std::cout << "Heff:\t" << Heff.transpose() << '\n';
            //std::cout << "Teff:\t" << Teff.transpose() << '\n';

            const auto& a = mNeelParams.NeelFactor1;
            const auto& b = mNeelParams.NeelFactor2;
            const auto& c = mBrownParams.BrownPrefactor;
            const auto d = c * MagneticMoment;

            DeterministicType result;
            auto Brown{ result.template head<3>() };
            auto Neel{ result.template tail<3>() };

            const auto mxHeff = ei.cross(Heff).eval();

            /* BEGIN Brown Rotation*/
            const auto omegabrown = c * Teff + d * mxHeff;
            Brown = omegabrown.cross(ni);
            /* END Brown Rotation*/

            /* BEGIN Neel Rotation*/
            const auto omeganeel = -a * Heff + (b + d)*mxHeff + c * Teff;
            Neel = omeganeel.cross(ei);
            /* END Neel Rotation*/

            return result;
        }


    }

    template<typename precision, typename aniso, bool SimpleModel>
    BASIC_ALWAYS_INLINE void BrownAndNeelRelaxation<precision, aniso, SimpleModel>::finishCalculations(DependentType& /* yi */) const noexcept
    {}

    template<typename precision, typename aniso, bool SimpleModel>
    inline decltype(auto) BrownAndNeelRelaxation<precision, aniso, SimpleModel>::getStart(const InitSettings& Init) const noexcept
    {
        DependentType Result;

        std::random_device rd; // Komplett nicht deterministisch aber langsam; Seed for faster generators only used sixth times here so it is ok
        std::normal_distribution<precision> nd{ 0,1 };

        //for (unsigned int i = 0; i < dim::NumberOfDependentVariables;++i)
        //	tmpvec(i) = nd(rd);

        if (Init.getUseRandomInitialParticleOrientation())
        {
            Vec3D Orientation;
            for (unsigned int i = 0; i < 3; ++i)
                Orientation(i) = nd(rd);
            Result.template head<3>() = Orientation;
        }
        else
        {
            Vec3D EulerAngles = Init.getInitialParticleOrientation();
            Vec3D Orientation;
            Orientation << 1.0, 0.0, 0.0;
            Matrix3x3	tmp;
            const auto &a = EulerAngles[0]; //!< Alpha
            const auto &b = EulerAngles[1];	//!< Beta
            const auto &g = EulerAngles[2]; //!< Gamma
            tmp << cos(a)*cos(g) - sin(a)*cos(b)*sin(g), sin(a)*cos(g) + cos(a)*cos(b)*sin(g), sin(b)*sin(g),
                -cos(a)*sin(g) - sin(a)*cos(b)*cos(g), -sin(a)*sin(g) + cos(a)*cos(b)*cos(g), sin(b)*cos(g),
                sin(a)*sin(b), -cos(a)*sin(b), cos(b);
            Result.template head<3>() = tmp*Orientation;
        }

        if (Init.getUseRandomInitialMagnetisationDir())
        {
            Vec3D MagDir;
            for (unsigned int i = 0; i < 3; ++i)
                MagDir(i) = nd(rd);
            Result.template tail<3>() = MagDir;
        }
        else
        {
            Result.template tail<3>() = Init.getInitialMagnetisationDirection();
        }

        normalize(Result); //normalize if necessary
        return Result;
    }
}
#endif //_BROWNANDNEELRELAXATION_H_

