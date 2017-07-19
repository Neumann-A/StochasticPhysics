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

#include "Properties/ParticleProperties.h"

///-------------------------------------------------------------------------------------------------
/// <summary>	Class to describe (magnetic) uniaxial anisotropy of first order. </summary>
///
/// <typeparam name="prec">	Floating point type </typeparam>
///-------------------------------------------------------------------------------------------------
template <typename prec>
class UniaxialAnisotropy
{
	static_assert(std::is_floating_point_v<prec>, "UniaxialAnisotropy: Template parameter must be floating point!");
private:
	typedef typename Eigen::Matrix<prec, 3, 1> InputVector;
	typedef typename Eigen::Matrix<prec, 3, 3> InputMatrix;
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
		return ((prefactor*ei.dot(ni))*ni).eval();
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


#endif	// INC_UniaxialAnisotropy_H
// end of Problems\Anisotropy\UniaxialAnisotropy.h
///---------------------------------------------------------------------------------------------------
