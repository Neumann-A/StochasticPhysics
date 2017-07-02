/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/
#pragma once

#ifndef _NOISEFIELD_H_
#define _NOISEFIELD_H_

#include "GeneralField.h"
#include <random>
#include <array>

#ifdef USE_BOOST
#include <boost/random/normal_distribution.hpp>
//#include <boost/random.hpp>
#endif

struct NoiseFieldParameters
{
	
};

template <typename prec, int dim, typename generator>
class NoiseField : public GeneralField< NoiseField<prec, dim, generator>>
{
public:
	using ThisClass = NoiseField<prec, dim, generator>;
	using Precision = prec;
	
	//template<typename Derived>
	//using GeneralBase = GeneralField<Derived>;

	using Base = GeneralField<ThisClass>;
	
	using Traits = typename Base::Traits;
	using FieldProperties = typename Traits::FieldProperties;
	using FieldVector = typename Traits::FieldVector;

#ifdef USE_BOOST
	using NormalDistribution = boost::random::normal_distribution<prec>;
#else
	using NormalDistribution = std::normal_distribution<prec>;
#endif

protected:
	constexpr NoiseField() = default;

private:
	std::array<generator, dim> m_generators;
	//std::array<NormalDistribution, dim> m_distributions;
	NormalDistribution m_distribution;

	void initGenerators(const std::size_t& NumberOfInit);
	void initGenerator(generator& gen, const std::size_t& NumberOfInit);

public:
	NoiseField(const std::size_t& NumberOfInit, const Precision& timestep);

	BASIC_ALWAYS_INLINE auto getField()-> FieldVector;
};

#include "NoiseField.inl"

#include "../Properties/FieldProperties.h"
#include <Eigen/Core>
#include <Eigen/StdVector>		// for Eigen std::vector allocator
template<typename prec, int dim, typename generator >
class FieldTraits<NoiseField<prec, dim, generator>>
{
public:
	using Precision = prec;
	using FieldProperties = Properties::FieldProperties<Precision>;
	using FieldVector = Eigen::Matrix<Precision, dim, 1>;
	using FieldVectorStdAllocator = Eigen::aligned_allocator<FieldVector>;
};

#endif //_NOISEFIELD_H_


