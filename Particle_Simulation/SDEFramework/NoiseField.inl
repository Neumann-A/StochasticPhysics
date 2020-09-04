/*
* Author: Alexander Neumann
* Date : 23.08.2015
*/

#pragma once

#include <utility>
#include <MyCEL/math/random_helpers.h>

template<typename prec, int dim, typename generator, typename NormalDistribution>
inline NoiseField<prec, dim, generator, NormalDistribution>::NoiseField(const std::size_t& NumberOfInit, const Precision& timestep)
{
    std::random_device dev;
	// Create Random Number Generators
	for(auto& gen : m_generators)
	{
        gen = math::random_helpers::create_seeded_PRNG<generator>(dev);
		m_distribution = NormalDistribution{ 0, sqrt(timestep) };
	}
	initGenerators(NumberOfInit);
};

template<typename prec, int dim, typename generator, typename NormalDistribution>
BASIC_ALWAYS_INLINE auto NoiseField<prec, dim, generator, NormalDistribution>::getField() -> FieldVector
{

	//std::array<prec, dim> values;
	//auto rand = [](auto& gen, auto& dist) { return dist(gen); };
	//std::transform(m_generators.begin(), m_generators.end(), m_distributions.begin(), values.begin(), rand);

	//FieldVector tmp = Eigen::Map<FieldVector>(values.data());
	FieldVector tmp;
	typename FieldVector::Index counter{ 0 };
	for (auto& gen : m_generators)
	{
		tmp(counter, 0) = m_distribution(gen);
		++counter;
	}
	//auto iter = m_generators.begin();
	//for (int i = dim; i--; ++iter)
	//	tmp(i, 0) = m_distribution(*iter);
	return tmp; //Fastest code so far; does not help to introduce a class member tmp
};

//template<typename prec, typename generator>
//inline auto NoiseField<prec, 6, generator>::getField()
//{
//	FieldVector tmp;
//	tmp(0, 0) = m_distribution(m_generators[0]);
//	tmp(1, 0) = m_distribution(m_generators[1]);
//	tmp(2, 0) = m_distribution(m_generators[2]);
//	tmp(3, 0) = m_distribution(m_generators[3]);
//	tmp(4, 0) = m_distribution(m_generators[4]);
//	tmp(5, 0) = m_distribution(m_generators[5]);
//	return tmp; //Fastest code so far; does not help to introduce a class member tmp
//};

template<typename prec, int dim, typename generator, typename NormalDistribution>
inline void NoiseField<prec, dim, generator, NormalDistribution>::initGenerators(const std::size_t& NumberOfInit)
{
	//std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> field_init_begin, field_init_finished;
	//field_init_begin = std::chrono::high_resolution_clock::now();
	for(auto &gen : this->m_generators)
	{
		this->initGenerator(gen, NumberOfInit);
	}
	//field_init_finished = std::chrono::high_resolution_clock::now();
	//std::cout << "It took " << (field_init_finished - field_init_begin).count() / 1E6 << " ms to initialize the Generators for the NoiseField " << std::endl;

};

template<typename prec, int dim, typename generator, typename NormalDistribution>
inline void NoiseField<prec, dim, generator, NormalDistribution>::initGenerator(generator& gen, const std::size_t& NumberOfInit)
{
	gen.discard(NumberOfInit);
};

