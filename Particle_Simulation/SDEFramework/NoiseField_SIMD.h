/*    
* Author: Alexander Neumann
* Date : 23.08.2015
*/
#pragma once

#ifndef INC_NOISEFIELD_SIMD_H_
#define INC_NOISEFIELD_SIMD_H_

#include "GeneralField.h"
#include <random>
#include <Eigen\Dense>

//struct NoiseFieldParameters{};
#include <simd_rand/simd_normal_dist.h>
#include <simd_rand/simd_lcg.h>
namespace {
	constexpr const auto used_arch = array_simd_pcg::InstructionSet::AVX2;
}
template <typename prec, int dim, typename generator = array_simd_pcg::packed_PCG<dim,1024, used_arch>, typename NormalDistribution = array_simd_pcg::norm_dist_rng<generator, prec>>
class NoiseField_SIMD : public GeneralField< NoiseField_SIMD<prec, dim, generator, NormalDistribution>>
{
public:
	using ThisClass = NoiseField_SIMD<prec, dim, generator>;
	using Precision = prec; 

	using Base = GeneralField<ThisClass>;
	
	using Traits = typename Base::Traits;
	using FieldVector = typename Traits::FieldVector;

	using Distribution = NormalDistribution;

protected:
	constexpr NoiseField_SIMD() = default;

private:
	generator m_generators;
	NormalDistribution m_distribution;
	
	int offset = 0;
	static constexpr const int results_per_gen = generator::calulated_results_per_generator; // 4
	static constexpr const int dimension = dim; // 6
	static constexpr const int size_of_internal_storage = results_per_gen * dim;
	Precision *res;

public:
	NoiseField_SIMD(const std::size_t& NumberOfInit, const Precision& timestep);
	//NoiseField(const std::size_t& NumberOfInit, const FieldVector& NoisePrefactor);

	inline auto getField()-> FieldVector;
};

#include <Eigen/Core>
#include <Eigen/StdVector>		// for Eigen std::vector allocator
template<typename prec, int dim, typename generator, typename NormalDistribution >
class FieldTraits<NoiseField_SIMD<prec, dim, generator, NormalDistribution>>
{
public:
	using Precision = prec;
	using FieldVector = Eigen::Matrix<Precision, dim, 1>;
	using FieldVectorStdAllocator =  std::allocator<FieldVector>;
};

#include "NoiseField_SIMD.inl"

#endif //INC_NOISEFIELD_SIMD_H_
