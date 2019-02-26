/*    
* Author: Alexander Neumann
* Date : 23.08.2015
*/
#pragma once

#ifndef INC_NOISEFIELD_H_
#define INC_NOISEFIELD_H_

#include "GeneralField.h"
#include <random>
#include <array>

#ifdef USE_BOOST_RANDOM
#include <boost/random/normal_distribution.hpp>
#endif

struct NoiseFieldParameters{};

#ifdef USE_BOOST_RANDOM
template<typename prec>
using DefaultNormalDistribution = boost::random::normal_distribution<prec>;
#else
template<typename prec>
using DefaultNormalDistribution = std::normal_distribution<prec>;
#endif


template <typename prec, int dim, typename generator, typename NormalDistribution = DefaultNormalDistribution<prec>>
class NoiseField : public GeneralField< NoiseField<prec, dim, generator, NormalDistribution>>
{
public:
	using ThisClass = NoiseField<prec, dim, generator>;
	using Precision = prec;
	
	//template<typename Derived>
	//using GeneralBase = GeneralField<Derived>;

	using Base = GeneralField<ThisClass>;
	
	using Traits = typename Base::Traits;
	using FieldVector = typename Traits::FieldVector;

	using Distribution = NormalDistribution;

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
	NoiseField(const std::size_t& NumberOfInit, const FieldVector& NoisePrefactor);

	BASIC_ALWAYS_INLINE auto getField()-> FieldVector;
};

#include "NoiseField.inl"

#include <Eigen/Core>
//#include <Eigen/StdVector>		// for Eigen std::vector allocator
template<typename prec, int dim, typename generator, typename NormalDistribution >
class FieldTraits<NoiseField<prec, dim, generator, NormalDistribution>>
{
public:
	using Precision = prec;
	using FieldVector = Eigen::Matrix<Precision, dim, 1>;
	using FieldVectorStdAllocator =  std::allocator<FieldVector>;
};
#endif //_NOISEFIELD_H_
