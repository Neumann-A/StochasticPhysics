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
#include <cstdint>

#include "GeneralAnisotropy.h"
#include "Properties/ParticleProperties.h"
#include "AnisotropyList.h"

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

	public:
		using Precision = prec;
		using traits = typename BaseClass::traits;
		template<typename T>
		using BaseVector = typename traits::template BaseVector<T>;
		using InputVector = typename traits::InputVector;
		using OutputVector = typename traits::OutputVector;
		using InputMatrix = Eigen::Matrix<prec, 3, 3>;
		using JacobiMatrix = typename traits::JacobiMatrix;
	private:

		//Prefactors
		const prec K1_MS;
		const prec K1VM;

		//Precalculated Cache!
		struct Cache
		{
			prec c1m;
			prec c2m;
			prec c3m;
			prec c1m_2;
			prec c2m_2;
			prec c3m_2;
		};

		mutable Cache pre;

	public:
		CubicAnisotropy(const Properties::MagneticProperties<prec>& MagProps) :
			K1_MS(-2.0*MagProps.getAnisotropyConstants().at(0)/ MagProps.getSaturationMagnetisation()),
			K1VM(-2.0*MagProps.getAnisotropyConstants().at(0)*MagProps.getMagneticVolume())
		{

		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Prepares the field by precalculating some values </summary>
		///
		/// <param name="ei"> 	The magnetization direction </param>
		/// <param name="xi"> 	The first body axis </param>
		/// <param name="yi"> 	The second body axis </param>
		/// <param name="zi"> 	The third body axis </param>
		///
		/// <returns>	The effective field. </returns>
		///-------------------------------------------------------------------------------------------------
		template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis>
		BASIC_ALWAYS_INLINE void prepareField(const BaseVector<MUnit> &ei,
			const BaseVector<XAxis> &xi,
			const BaseVector<YAxis> &yi,
			const BaseVector<ZAxis> &zi) const noexcept
		{
			pre.c1m = ei.dot(xi);
			pre.c2m = ei.dot(yi);
			pre.c3m = ei.dot(zi);
			pre.c1m_2 = pre.c1m * pre.c1m;
			pre.c2m_2 = pre.c2m * pre.c2m;
			pre.c3m_2 = pre.c3m * pre.c3m;
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
			const auto term1 = K1_MS*((pre.c2m_2 + pre.c3m_2)*pre.c1m*xi + (pre.c1m_2 + pre.c3m_2)*pre.c2m*yi + (pre.c1m_2 + pre.c2m_2)*pre.c3m*zi);
			//const auto term2 = K2_MS*(c2m_2*c3m_2*c1mxi + c1m_2*c3m_2*c2myi + c1m_2*c2m_2*c3mzi);
			//const auto term3 = K3_MS*((c2m_4+ c3m_4)*c1m_2*c1mxi + (c1m_4 + c3m_4)*c2m_2*c2myi + (c1m_4 + c2m_4)*c3m_2*c3mzi);
			
			//const auto sum = term1 + term2 + term3;
			const auto sum = term1;
			return sum.eval();
		};


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
		template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis, typename Euler, typename Sines, typename Cosines>
		NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei,
			const BaseVector<XAxis> &xi,
			const BaseVector<YAxis> &yi,
			const BaseVector<ZAxis> &zi,
			const BaseVector<Euler> &ori,
			const BaseVector<Sines> &statesin,
			const BaseVector<Cosines> &statecos) const noexcept
		{
			const auto& m = ei;
			const auto& c1 = xi;
			const auto& c2 = yi;
			const auto& c3 = zi;

			const auto& cphi = statecos(0);
			const auto& ctheta = statecos(1);
			const auto& cpsi = statecos(2);
			const auto& sphi = statesin(0);
			const auto& stheta = statesin(1);
			const auto& spsi = statesin(2);

			const InputVector theta(ctheta*spsi,-ctheta*cpsi,-stheta);
			const InputVector phi(cpsi,spsi,0.0);

			const auto symetric = m*(pre.c3m)*(pre.c1m_2+pre.c2m_2);
			const auto symetricresult = c3.cross(symetric);

			const auto asymetricdir = (theta*ctheta*1.0/stheta + c3);
			const auto asymetricterm = pre.c1m*pre.c2m*(pre.c2m_2-pre.c1m_2);
			const auto asymetricresult = asymetricterm* asymetricdir;
			return (K1VM*(symetricresult + asymetricresult)).eval();
		};

		//template<typename MUnit, typename XAxis, typename YAxis, typename ZAxis, typename Euler, typename Sines, typename Cosines>
		/////-------------------------------------------------------------------------------------------------
		///// <summary>	Gets the effective torque for this anisotropy </summary>
		/////
		///// <param name="ei"> 	The magnetization direction </param>
		///// <param name="xi"> 	The first body axis </param>
		///// <param name="yi"> 	The second body axis </param>
		///// <param name="zi"> 	The third body axis </param>
		///// <param name="ori">	Orientation of particle (Euler angles) </param>
		///// <param name="sin">	Pre-calculated sines of Euler angles </param>
		///// <param name="cos">	Pre-calculated cosines of Euler angles </param>
		/////
		///// <returns>	The effective torque. </returns>
		/////-------------------------------------------------------------------------------------------------
		//NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei,
		//	const BaseVector<XAxis> &xi,
		//	const BaseVector<YAxis> &yi,
		//	const BaseVector<ZAxis> &zi,
		//	const BaseVector<Euler> &ori,
		//	const BaseVector<Sines> &sin,
		//	const BaseVector<Cosines> &cos) const noexcept
		//{
		//	return getEffTorque(ei, zi);
		//};

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
			//TODO: implement Jacobi!
			return JacobiMatrix::Identity();
		};

	private:
		//template<typename MUnit, typename EasyAxis>
		//NODISCARD BASIC_ALWAYS_INLINE auto getAnisotropyField(const BaseVector<MUnit> &ei, const BaseVector<EasyAxis> &ni) const noexcept
		//{
		//	return ((prefactorField*ei.dot(ni))*ni);
		//};

		//template<typename MUnit, typename EasyAxis>
		//NODISCARD BASIC_ALWAYS_INLINE auto getEffTorque(const BaseVector<MUnit> &ei, const BaseVector<EasyAxis> &ni) const noexcept
		//{
		//	return ni.cross((prefactorTorque*ei.dot(ni))*ei);
		//};

		};

	template<typename prec>
	struct AnisotropyTraits<CubicAnisotropy<prec>>
	{
		//Default Traits:
		using Precision = prec;
		using Anisotropy = CubicAnisotropy<Precision>;
		using InputVector = Eigen::Matrix<Precision, 3, 1>;
		using OutputVector = Eigen::Matrix<Precision, 3, 1>;
		using JacobiMatrix = Eigen::Matrix<Precision, 3, 3>;
		template<typename T>
		using BaseVector = Eigen::MatrixBase<T>;

		static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
		static constexpr bool is_specialized_v = true;
		static constexpr std::uint8_t number_anisotropy_constants = 1;

		using value_type = Properties::IAnisotropy;
		static constexpr value_type value = value_type::Anisotropy_cubic;
	};

}
#endif	// INC_CubicAnisotropy_H
// end of Problems\Anisotropy\CubicAnisotropy.h
///---------------------------------------------------------------------------------------------------
