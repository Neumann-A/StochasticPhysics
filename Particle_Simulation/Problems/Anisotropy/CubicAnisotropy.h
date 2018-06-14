///---------------------------------------------------------------------------------------------------
// file:		Problems\Anisotropy\CubicAnisotropy.h
//
// summary: 	Declares the cubic anisotropy class
//
// Copyright (c) 2018 Alexander Neumann.
//
// author: Alexander Neumann
// date: 08.06.2018

#ifndef INC_CubicAnisotropy_H
#define INC_CubicAnisotropy_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <Eigen/Core>
#include <type_traits>

#include "GeneralAnisotropy.h"
#include "Properties/ParticleProperties.h"

namespace Problems::Anisotropy
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	Class to describe (magnetic) uniaxial anisotropy of first order. </summary>
	///
	/// <typeparam name="prec">	Floating point type </typeparam>
	///-------------------------------------------------------------------------------------------------
	template <typename prec>
	class CubicAnisotropy : public GeneralAnisotropy<CubicAnisotropy<prec>>
	{
		using ThisClass = CubicAnisotropy<prec>;
		using BaseClass = GeneralAnisotropy<ThisClass>;
		using traits = typename BaseClass::traits;
	public:
		using Precision = prec;
		template<typename T>
		using BaseVector = typename traits::template BaseVector<T>;
		using InputVector = typename traits::InputVector;
		using OutputVector = typename traits::OutputVector;
		using InputMatrix = Eigen::Matrix<prec, 3, 3>;
	private:
		//There are two possibilities to calculated the pre-factors here
		// 1. Precalculated all values -> means store 6 values;
		// 2. Precalculate nothing -> store 5 values; do one multiplication and one extra div at runtime
		// I went with the first approach
		const prec K1_MS;
		const prec K2_MS;
		const prec K3_MS;
		const prec K1VM;
		const prec K2VM;
		const prec K3VM;

	public:
		CubicAnisotropy(const Properties::MagneticProperties<prec>& MagProps) :
			K1_MS(-2.0*MagProps.getAnisotropyConstants().at(0)/ MagProps.getSaturationMagnetisation()),
			K2_MS(-2.0*MagProps.getAnisotropyConstants().at(1)/ MagProps.getSaturationMagnetisation()),
			K3_MS(-4.0*MagProps.getAnisotropyConstants().at(2)/ MagProps.getSaturationMagnetisation()),
			K1VM(-2.0*MagProps.getAnisotropyConstants().at(0)*MagProps.getMagneticVolume()),
			K2VM(-2.0*MagProps.getAnisotropyConstants().at(1)*MagProps.getMagneticVolume()),
			K3VM(-4.0*MagProps.getAnisotropyConstants().at(2)*MagProps.getMagneticVolume())
		{

		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the effective field for this anisotropy </summary>
		///
		/// <param name="ei"> 	The magnetization direction </param>
		/// <param name="xi"> 	The first body axis </param>
		/// <param name="yi"> 	The second body axis </param>
		/// <param name="zi"> 	The third body axis </param>
		///
		/// <returns>	The effective field. </returns>
		///-------------------------------------------------------------------------------------------------

		template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
		NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const BaseVector<MUnit> &ei,
			const BaseVector<XAxis> &xi,
			const BaseVector<YAxis> &yi,
			const BaseVector<ZAxis> &zi) const noexcept
		{		
			const auto c1m = xi.dot(ei).eval();
			const auto c2m = yi.dot(ei).eval();
			const auto c3m = zi.dot(ei).eval();
			const auto c1mxi = c1m * xi;
			const auto c2myi = c2m * yi;
			const auto c3mzi = c3m * zi;
			const auto c1m_2 = c1m * c1m;
			const auto c2m_2 = c2m * c2m;
			const auto c3m_2 = c3m * c3m;
			const auto c1m_4 = c1m_2 * c1m_2;
			const auto c2m_4 = c2m_2 * c2m_2;
			const auto c3m_4 = c3m_2 * c3m_2;

			const auto term1 = K1_MS*((c2m_2 + c3m_2)*c1mxi + (c1m_2 + c3m_2)*c2myi + (c1m_2 + c2m_2)*c3mzi);
			const auto term2 = K2_MS*(c2m_2*c3m_2*c1mxi + c1m_2*c3m_2*c2myi + c1m_2*c2m_2*c3mzi);
			const auto term3 = K3_MS*((c2m_4+ c3m_4)*c1m_2*c1mxi + (c1m_4 + c3m_4)*c2m_2*c2myi + (c1m_4 + c2m_4)*c3m_2*c3mzi);
			
			const auto sum = term1 + term2 + term3;
			return sum.eval();
		};

		template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis, typename Euler, typename Sines, typename Cosines>

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the effective torque for this anisotropy </summary>
		///
		/// <param name="ei"> 	The magnetization direction </param>
		/// <param name="xi"> 	The first body axis </param>
		/// <param name="yi"> 	The second body axis </param>
		/// <param name="zi"> 	The third body axis </param>
		/// <param name="ori">	Orientation of particle (Euler angles) </param>
		/// <param name="sin">	Pre-calculated sines of Euler angles </param>
		/// <param name="cos">	Pre-calculated cosines of Euler angles </param>
		///
		/// <returns>	The effective torque. </returns>
		///-------------------------------------------------------------------------------------------------
		NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei,
			const BaseVector<XAxis> &xi,
			const BaseVector<YAxis> &yi,
			const BaseVector<ZAxis> &zi,
			const BaseVector<Euler> &ori,
			const BaseVector<Sines> &sin,
			const BaseVector<Cosines> &cos) const noexcept
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
			return BaseVector<ZAxis>::Identity();
		};

	private:
		template<typename MUnit, typename EasyAxis>
		NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const BaseVector<MUnit> &ei, const BaseVector<EasyAxis> &ni) const noexcept
		{
			return ((prefactorField*ei.dot(ni))*ni);
		};

		template<typename MUnit, typename EasyAxis>
		NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei, const BaseVector<EasyAxis> &ni) const noexcept
		{
			return ni.cross((prefactorTorque*ei.dot(ni))*ei);
		};

		};

	template<typename prec>
	struct AnisotropyTraits<CubicAnisotropy<prec>>
	{
		//Default Traits:
		using Precision = prec;
		using Anisotropy = UniaxialAnisotropy<Precision>;
		using InputVector = Eigen::Matrix<Precision, 3, 1>;
		using OutputVector = Eigen::Matrix<Precision, 3, 1>;
		template<typename T>
		using BaseVector = Eigen::MatrixBase<T>;

		static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
		static constexpr bool is_specialized_v = true;
		static constexpr bool needed_anisotropies = 3;
	};

}
#endif	// INC_CubicAnisotropy_H
// end of Problems\Anisotropy\CubicAnisotropy.h
///---------------------------------------------------------------------------------------------------
