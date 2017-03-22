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
	const prec prefactor;
	const prec AnisotropyConstant;
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	UniaxialAnisotropy(const prec aniso, const prec sat) : prefactor(2 * aniso / (sat)), AnisotropyConstant(aniso) {};
	UniaxialAnisotropy(const prec sat, const std::vector<prec> aniso) : prefactor(2 * (*(aniso.begin())) / (sat)), AnisotropyConstant(*(aniso.begin())) {};
	UniaxialAnisotropy(const Properties::ParticlesProperties<prec>& ParProps) :
		prefactor(2 * (*(ParProps.getMagneticProperties().getAnisotropyConstants().begin())) / (ParProps.getMagneticProperties().getSaturationMagnetisation())),
		AnisotropyConstant(*(ParProps.getMagneticProperties().getAnisotropyConstants().begin()))
	{};

	///-------------------------------------------------------------------------------------------------
	/// <summary>	Gets the effective field. </summary>
	///
	/// <param name="ei">	The normailized (magnetisation) vector. </param>
	/// <param name="ni">	The direction of the preferred axis </param>
	///
	/// <returns>	The effective field. </returns>
	///-------------------------------------------------------------------------------------------------
	BASIC_ALWAYS_INLINE auto getEffectiveField(const InputVector &ei, const InputVector &ni) const
	{
		return ((prefactor*ei.dot(ni))*ni);
	};
};


#endif	// INC_UniaxialAnisotropy_H
// end of Problems\Anisotropy\UniaxialAnisotropy.h
///---------------------------------------------------------------------------------------------------
