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
#include "Properties/ParticleProperties.h"

namespace Problems::Anisotropy
{
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
		using traits	= typename BaseClass::traits;
	public:
		using Precision = prec;
		using InputVector = typename traits::InputVector;
		using InputMatrix = Eigen::Matrix<prec, 3, 3>;
	private:
		const prec prefactor; // 2K/MS
		const prec prefactor2; // 2*K*VM

		static BASIC_ALWAYS_INLINE auto calcPrefactor1(const Properties::MagneticProperties<prec>& MagProps) noexcept
		{
			const auto K_Uni = MagProps.getAnisotropyConstants().at(0);
			const auto M_S = MagProps.getSaturationMagnetisation();

			return (-2.0 * K_Uni / M_S);
		}

		static BASIC_ALWAYS_INLINE auto calcPrefactor2(const Properties::MagneticProperties<prec>& MagProps) noexcept
		{
			const auto K_Uni = MagProps.getAnisotropyConstants().at(0);
			const auto V_Mag = MagProps.getMagneticVolume();

			return (-2.0 * K_Uni * V_Mag);
		}

	public:
		UniaxialAnisotropy(const Properties::MagneticProperties<prec>& MagProps) :
			prefactor(calcPrefactor1(MagProps)), prefactor2(calcPrefactor2(MagProps))
		{};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets the effective field. </summary>
		///
		/// <param name="ei">	The normalized (magnetisation) vector. </param>
		/// <param name="ni">	The direction of the preferred axis </param>
		///
		/// <returns>	The effective field. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE auto getAnisotropyField(const InputVector &ei, const InputVector &ni) const
		{
			return ((prefactor*ei.dot(ni))*ni);
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets anisotropy field. Overload for maybe faster problem evaluation </summary>
		///
		/// <param name="ei">	The normalized (magnetisation) vector. </param>
		/// <param name="ni">	The direction of the preferred axis </param>
		/// <param name="eidotni">	ei.dot(ni). </param>
		///
		/// <returns>	The anisotropy field. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE auto getAnisotropyField(const InputVector &, const InputVector &ni, const prec& eidotni) const
		{
			return ((prefactor*eidotni)*ni);
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets jacobi matrix of the anisotroy field. </summary>
		///
		/// <param name="ei">	The normalized (magnetisation) vector. </param>
		/// <param name="ni">	The direction of the preferred axis </param>
		///
		/// <returns>	Jacobi matrix of anisotropy field. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE auto getJacobiAnisotropyField(const InputVector &, const InputVector &ni) const
		{
			return ((prefactor*ni)*ni.transpose()).eval();
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets jacobi matrix of the anisotroy field. </summary>
		///
		/// <param name="niouterni">	the outer product of ni. </param>
		///
		/// <returns>	Jacobi matrix of anisotropy field. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE auto getJacobiAnisotropyField(const InputMatrix& niouterni) const
		{
			return prefactor*niouterni;
		};

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets force field. </summary>
		///
		/// <param name="ei">	The normalized (magnetisation) vector. </param>
		/// <param name="ni">	The direction of the preferred axis </param>
		///
		/// <returns>	The force field. </returns>
		///-------------------------------------------------------------------------------------------------
		BASIC_ALWAYS_INLINE auto getForceField(const InputVector &ei, const InputVector &ni) const
		{
			return ((prefactor2*ei.dot(ni))*ei);
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
		BASIC_ALWAYS_INLINE auto getForceField(const InputVector &ei, const InputVector &, const prec& eidotni) const
		{
			return ((prefactor2*eidotni)*ei);
		};

		BASIC_ALWAYS_INLINE auto getJacobiForceField(const InputVector &ei, const InputVector &) const
		{
			return (prefactor2*ei)*ei.transpose();
		};

		BASIC_ALWAYS_INLINE auto getJacobiForceField(const InputMatrix& eiouterei) const
		{
			return (prefactor2*eiouterei);
		};
	};

	template<typename prec>
	struct AnisotropyTraits<UniaxialAnisotropy<prec>>
	{
		//Default Traits:
		using Precision = prec;
		using Anisotropy = UniaxialAnisotropy<Precision>;
		using InputVector = Eigen::Matrix<Precision, 3, 1>;

		static constexpr CoordinateSystem coordsystem = CoordinateSystem::cartesian;
		static constexpr bool is_specialized_v = true;
	};

}
#endif	// INC_UniaxialAnisotropy_H
// end of Problems\Anisotropy\UniaxialAnisotropy.h
///---------------------------------------------------------------------------------------------------
