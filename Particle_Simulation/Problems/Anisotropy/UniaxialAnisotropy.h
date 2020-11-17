///---------------------------------------------------------------------------------------------------
// file:		Problems\Anisotropy\UniaxialAnisotropy.h
//
// summary: 	Declares the uniaxial anisotropy class
//
// Copyright (c) 2017 Alexander Neumann.
//
// author: Neumann
// date: 23.08.2015

#ifndef INC_UniaxialAnisotropy_H
#define INC_UniaxialAnisotropy_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <Eigen/Core>
#include <type_traits>

#include "GeneralAnisotropy.h"
#include "Properties/Anisotropy/Uniaxial.hpp"
#include "Properties/ParticleProperties.h"

namespace Problems::Anisotropy
{
    template <typename prec>
    class UniaxialAnisotropy;

    template<typename prec>
    struct AnisotropyTraits<UniaxialAnisotropy<prec>>
    {
        //Default Traits:
        using Precision = prec;
        using Anisotropy = UniaxialAnisotropy<Precision>;
        using InputVector = Eigen::Matrix<Precision, 3, 1>;
        using OutputVector = Eigen::Matrix<Precision, 3, 1>;
        using JacobiMatrix = Eigen::Matrix<Precision, 3, 3>;
        template<typename T>
        using BaseVector = Eigen::MatrixBase<T>;

        static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
        static constexpr bool is_specialized_v = true;
        static constexpr std::uint8_t number_anisotropy_constants = 1;

        using value_type = Properties::IAnisotropy;
        static constexpr value_type value = value_type::Anisotropy_uniaxial;
    };

    ///-------------------------------------------------------------------------------------------------
    /// <summary>	Class to describe (magnetic) uniaxial anisotropy of first order. </summary>
    ///
    /// <typeparam name="prec">	Floating point type </typeparam>
    ///-------------------------------------------------------------------------------------------------
    template <typename prec>
    class UniaxialAnisotropy : public GeneralAnisotropy<UniaxialAnisotropy<prec>>
    {
        using ThisClass = UniaxialAnisotropy<prec>;
        using BaseClass = GeneralAnisotropy<ThisClass>;

    public:
        using Precision = prec;
        using traits = typename BaseClass::traits;
        template<typename T>
        using BaseVector = typename traits::template BaseVector<T>;
        using InputVector = typename traits::InputVector;
        using OutputVector = typename traits::OutputVector;
        using InputMatrix = Eigen::Matrix<prec, 3, 3>;
    private:
        const prec prefactorField; // -2K/MS
        const prec prefactorTorque; // -2*K*VM

        NODISCARD static BASIC_ALWAYS_INLINE auto calcEffFieldPrefactor(const Properties::MagneticProperties<prec>& MagProps) noexcept
        {
            const auto K_Uni = MagProps.template getAnisotropyProperties<traits::value>().K_uniaxial;
            const auto M_S = MagProps.getSaturationMagnetisation();

            return (-2.0 * K_Uni / M_S);
        }

        NODISCARD static BASIC_ALWAYS_INLINE auto calcEffTorquePrefactor(const Properties::MagneticProperties<prec>& MagProps) noexcept
        {
            const auto K_Uni = MagProps.template getAnisotropyProperties<traits::value>().K_uniaxial;
            const auto V_Mag = MagProps.getMagneticVolume();

            return (-2.0 * K_Uni * V_Mag);
        }

    public:
        UniaxialAnisotropy(const Properties::MagneticProperties<prec>& MagProps) :
            prefactorField(calcEffFieldPrefactor(MagProps)), prefactorTorque(calcEffTorquePrefactor(MagProps))
        {};

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        BASIC_ALWAYS_INLINE void prepareField(const BaseVector<MUnit> &,
            const BaseVector<XAxis> &,
            const BaseVector<YAxis> &,
            const BaseVector<ZAxis> &) const noexcept
        {
        };

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const BaseVector<MUnit> &ei,
                                                              const BaseVector<XAxis> &,
                                                              const BaseVector<YAxis> &,
                                                              const BaseVector<ZAxis> &zi) const noexcept
        {
            return getAnisotropyField(ei,zi);
        };

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis, typename Euler, typename Sines, typename Cosines>
        NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei,
                                                        const BaseVector<XAxis> &,
                                                        const BaseVector<YAxis> &,
                                                        const BaseVector<ZAxis> &zi,
                                                        const BaseVector<Euler> &,
                                                        const BaseVector<Sines> &,
                                                        const BaseVector<Cosines> &) const noexcept
        {
            return getEffTorque(ei, zi);
        };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Gets jacobi matrix of the anisotroy field. </summary>
        ///
        /// <param name="ei">	The normalized (magnetisation) vector. </param>
        /// <param name="ni">	The direction of the preferred axis </param>
        ///
        /// <returns>	Jacobi matrix of anisotropy field. </returns>
        ///-------------------------------------------------------------------------------------------------
        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        NODISCARD BASIC_ALWAYS_INLINE auto getJacobiAnisotropyField(const BaseVector<MUnit> &,
                                                                    const BaseVector<XAxis> &,
                                                                    const BaseVector<YAxis> &,
                                                                    const BaseVector<ZAxis> &zi) const noexcept
        {
            return ((prefactorField*zi)*zi.transpose()).eval();
        };

    private:
        template<typename MUnit, typename EasyAxis>
        NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const BaseVector<MUnit> &ei, const BaseVector<EasyAxis> &ni) const noexcept
        {
            return ((prefactorField*ei.dot(ni))*ni);
        };

        template<typename MUnit, typename EasyAxis>
        NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei,const BaseVector<EasyAxis> &ni) const noexcept
        {
            return ni.cross((prefactorTorque*ei.dot(ni))*ei);
        };

        //----------------------- old stuff for compatiblity --------------------------------------------//


        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Gets the effective field. </summary>
        ///
        /// <param name="ei">	The normalized (magnetisation) vector. </param>
        /// <param name="ni">	The direction of the preferred axis </param>
        ///
        /// <returns>	The effective field. </returns>
        ///-------------------------------------------------------------------------------------------------


        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Gets anisotropy field. Overload for maybe faster problem evaluation </summary>
        ///
        /// <param name="ei">	The normalized (magnetisation) vector. </param>
        /// <param name="ni">	The direction of the preferred axis </param>
        /// <param name="eidotni">	ei.dot(ni). </param>
        ///
        /// <returns>	The anisotropy field. </returns>
        ///-------------------------------------------------------------------------------------------------
        DEPRECATED NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const InputVector &, const InputVector &ni, const prec& eidotni) const
        {
            return ((prefactorField*eidotni)*ni);
        };



        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Gets jacobi matrix of the anisotroy field. </summary>
        ///
        /// <param name="niouterni">	the outer product of ni. </param>
        ///
        /// <returns>	Jacobi matrix of anisotropy field. </returns>
        ///-------------------------------------------------------------------------------------------------
        DEPRECATED NODISCARD BASIC_ALWAYS_INLINE auto getJacobiAnisotropyField(const InputMatrix& niouterni) const
        {
            return prefactorField*niouterni;
        };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Gets force field. </summary>
        ///
        /// <param name="ei">	The normalized (magnetisation) vector. </param>
        /// <param name="ni">	The direction of the preferred axis </param>
        ///
        /// <returns>	The force field. </returns>
        ///-------------------------------------------------------------------------------------------------
        DEPRECATED NODISCARD BASIC_ALWAYS_INLINE auto getForceField(const InputVector &ei, const InputVector &ni) const
        {
            return ((prefactorTorque*ei.dot(ni))*ei);
        };

        ///-------------------------------------------------------------------------------------------------
        /// <summary>	Gets force field. </summary>
        ///
        /// <param name="ei">	The normalized (magnetisation) vector. </param>
        /// <param name="ni">	The direction of the preferred axis </param>
        /// <param name="eidotni">	ei.dot(ni) for maybe faster problem evaluation. </param>
        ///
        /// <returns>	The force field. </returns>
        ///-------------------------------------------------------------------------------------------------
        DEPRECATED NODISCARD BASIC_ALWAYS_INLINE auto getForceField(const InputVector &ei, const InputVector &, const prec& eidotni) const
        {
            return ((prefactorTorque*eidotni)*ei);
        };

        DEPRECATED NODISCARD BASIC_ALWAYS_INLINE auto getJacobiForceField(const InputVector &ei, const InputVector &) const
        {
            return (prefactorTorque*ei)*ei.transpose();
        };

        DEPRECATED NODISCARD BASIC_ALWAYS_INLINE auto getJacobiForceField(const InputMatrix& eiouterei) const
        {
            return (prefactorTorque*eiouterei);
        };
    };
}
#endif	// INC_UniaxialAnisotropy_H
// end of Problems\Anisotropy\UniaxialAnisotropy.h
///---------------------------------------------------------------------------------------------------
