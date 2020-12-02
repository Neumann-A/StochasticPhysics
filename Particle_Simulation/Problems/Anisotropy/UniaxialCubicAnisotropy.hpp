#ifndef INC_UniaxialCubicAnisotropy_H
#define INC_UniaxialCubicAnisotropy_H

#pragma once

#include <type_traits>
#include "GeneralAnisotropy.h"
#include "Properties/ParticleProperties.h"

#include "UniaxialAnisotropy.h"
#include "CubicAnisotropy.h"

namespace Problems::Anisotropy
{
    template <typename prec>
    class UniaxialCubicAnisotropy;

    template<typename prec>
    struct AnisotropyTraits<UniaxialCubicAnisotropy<prec>>
    {
        //Default Traits:
        using Precision = prec;
        using Anisotropy = UniaxialCubicAnisotropy<Precision>;
        using InputVector = SPhys::math::Matrix<Precision, 3, 1>;
        using OutputVector = SPhys::math::Matrix<Precision, 3, 1>;
        using JacobiMatrix = SPhys::math::Matrix<Precision, 3, 3>;
        template<typename T>
        using BaseVector = SPhys::math::MatrixBase<T>;

        using input_parameter = ::Properties::Anisotropy::UniaxialCubic<prec>;

        static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
        static constexpr bool is_specialized_v = true;
        static constexpr std::uint8_t number_anisotropy_constants = 1;

        using value_type = Properties::IAnisotropy;
        static constexpr value_type value = value_type::Anisotropy_uniaxialcubic;
    };

    template <typename prec>
    class UniaxialCubicAnisotropy : public GeneralAnisotropy<UniaxialCubicAnisotropy<prec>>
    {
        using ThisClass = UniaxialCubicAnisotropy<prec>;
        using BaseClass = GeneralAnisotropy<ThisClass>;

    public:
        using Precision = prec;
        using traits = typename BaseClass::traits;
        template<typename T>
        using BaseVector = typename traits::template BaseVector<T>;
        using InputVector = typename traits::InputVector;
        using OutputVector = typename traits::OutputVector;
        using InputMatrix = SPhys::math::Matrix<prec, 3, 3>;
        using RotationMatrix = SPhys::math::Matrix<prec, 3, 3>;

    private:
        UniaxialAnisotropy uniaxial_part; // -2K/MS
        CubicAnisotropy cubic_part; // -2*K*VM
        RotationMatrix  cubic_orientation;

        mutable ei_rotated;

    public:
        UniaxialCubicAnisotropy(const Properties::MagneticProperties<prec>& MagProps) :
            uniaxial_part(MagProps), cubic_part(MagProps)
        {};

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        BASIC_ALWAYS_INLINE void prepareField(const BaseVector<MUnit> &ei,
            const BaseVector<XAxis> &xi,
            const BaseVector<YAxis> &yi,
            const BaseVector<ZAxis> &zi) const noexcept
        {
            uniaxial_part(cubic_orientation*ei,xi,yi,zi);
            cubic_part(ei,xi,yi,zi);
        };

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const BaseVector<MUnit> &ei,
                                                              const BaseVector<XAxis> &xi,
                                                              const BaseVector<YAxis> &yi,
                                                              const BaseVector<ZAxis> &zi) const noexcept
        {
            return uniaxial_part.getAnisotropyField(ei,xi,yi,zi)+;
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


    };
}

#endif
