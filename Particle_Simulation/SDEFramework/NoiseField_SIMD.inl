/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once

#include <utility>
#include <random>
#include <cmath>

template<typename prec, int dim, typename generator, typename NormalDistribution>
BASIC_ALWAYS_INLINE NoiseField_SIMD<prec, dim, generator, NormalDistribution>::NoiseField_SIMD(const std::size_t& NumberOfInit, const prec& timestep)
{
	std::random_device dev;
	// Create Random Number Generators
	m_generators = array_simd_pcg::create_Seeded_PRNG<generator>(dev);
	m_generators.discard(NumberOfInit);
	m_distribution = NormalDistribution{ m_generators ,sqrt(timestep) };

};

template<typename prec, int dim, typename generator, typename NormalDistribution>
BASIC_ALWAYS_INLINE auto NoiseField_SIMD<prec, dim, generator, NormalDistribution>::getField() -> FieldVector
{

	FieldVector tmp;
	if (offset == 0)
		res = m_distribution();
	tmp = Eigen::Map<FieldVector>(res + offset, dim);
	offset = (offset + dim) % size_of_internal_storage;

	return tmp;
};