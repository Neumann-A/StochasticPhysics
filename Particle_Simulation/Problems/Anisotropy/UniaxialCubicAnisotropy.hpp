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
        UniaxialAnisotropy<prec> uniaxial_part; // -2K/MS
        CubicAnisotropy<prec> cubic_part; // -2*K*VM
        RotationMatrix  uniaxial_rotation;

    [[nodiscard]] RotationMatrix calcRotation(const InputVector& euler_rotation) noexcept
    {
        RotationMatrix rot;
        const auto Sines = euler_rotation.array().sin().eval();
        const auto Cosines = euler_rotation.array().cos().eval();
        const auto& cphi    = Cosines(0);
        const auto& ctheta    = Cosines(1);
        const auto& cpsi    = Cosines(2);
        const auto& sphi    = Sines(0);
        const auto& stheta    = Sines(1);
        const auto& spsi    = Sines(2);

        //Phi and Psi products (used twice)
        const auto cphicpsi = cphi*cpsi;
        const auto sphicpsi = sphi*cpsi;
        const auto cphispsi = cphi*spsi;
        const auto sphispsi = sphi*spsi;

        // R313 Rotationmatrix transposed
        rot(0, 0) =  cphicpsi - ctheta*sphispsi;
        rot(0, 1) = -sphicpsi - ctheta*cphispsi;
        rot(0, 2) =  stheta*spsi;
        rot(1, 0) =  ctheta*sphicpsi + cphispsi;
        rot(1, 1) =  ctheta*cphicpsi - sphispsi;
        rot(1, 2) = -stheta*cpsi;
        rot(2, 0) =  stheta*sphi;
        rot(2, 1) =  stheta*cphi;
        rot(2, 2) =  ctheta;
        return rot;
    }
    public:
        UniaxialCubicAnisotropy(const Properties::MagneticProperties<prec>& MagProps) :
            uniaxial_part(MagProps.template getAnisotropyProperties<traits::value>().uniaxial,MagProps),
            cubic_part(MagProps.template getAnisotropyProperties<traits::value>().cubic,MagProps),
            uniaxial_rotation(calcRotation(MagProps.template getAnisotropyProperties<traits::value>().uniaxial_rotation))
        {};

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        BASIC_ALWAYS_INLINE void prepareField(const BaseVector<MUnit> &ei,
            const BaseVector<XAxis> &xi,
            const BaseVector<YAxis> &yi,
            const BaseVector<ZAxis> &zi) const noexcept
        {
            uniaxial_part.prepareField((uniaxial_rotation*ei).eval(),xi,yi,zi);   //does nothing !
            cubic_part.prepareField(ei,xi,yi,zi);                        // does some precalculations!
        };

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
        NODISCARD BASIC_ALWAYS_INLINE decltype(auto) getAnisotropyField(const BaseVector<MUnit> &ei,
                                                                        const BaseVector<XAxis> &xi,
                                                                        const BaseVector<YAxis> &yi,
                                                                        const BaseVector<ZAxis> &zi) const noexcept
        {
            return (uniaxial_part.getAnisotropyField(ei,xi,yi,(uniaxial_rotation.transpose()*zi).eval())+cubic_part.getAnisotropyField(ei,xi,yi,zi));
        };

        template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis, typename Euler, typename Sines, typename Cosines>
        NODISCARD BASIC_ALWAYS_INLINE decltype(auto) getEffTorque(const BaseVector<MUnit> &ei,
                                                                  const BaseVector<XAxis> &xi,
                                                                  const BaseVector<YAxis> &yi,
                                                                  const BaseVector<ZAxis> &zi,
                                                                  const BaseVector<Euler> &eui,
                                                                  const BaseVector<Sines> &sines,
                                                                  const BaseVector<Cosines> &cosines) const noexcept
        {
            return (uniaxial_part.getEffTorque((uniaxial_rotation*ei).eval(),xi,yi,zi,eui,sines,cosines)+cubic_part.getEffTorque(ei,xi,yi,zi,eui,sines,cosines));
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
        NODISCARD BASIC_ALWAYS_INLINE auto getJacobiAnisotropyField(const BaseVector<MUnit> &ei,
                                                                    const BaseVector<XAxis> &xi,
                                                                    const BaseVector<YAxis> &yi,
                                                                    const BaseVector<ZAxis> &zi) const noexcept
        {
            return uniaxial_part.getJacobiAnisotropyField(ei,xi,yi,zi) + cubic_part.getJacobiAnisotropyField(ei,xi,yi,zi);
        };

    private:


    };
}

#endif
