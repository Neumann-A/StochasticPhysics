///-------------------------------------------------------------------------------------------------
// file:	BM_NoiseVector_F.cpp
//
// summary:	Implements the bm noise vector f class
///-------------------------------------------------------------------------------------------------
#include "BM_NoiseVector_F.h"

#ifdef USE_PCG_RANDOM
#include <pcg_random.hpp>
#endif
#ifdef USE_BOOST_RANDOM
#include <boost/random.hpp>
#endif


namespace
{
	using Precision = typename BM_NoiseVector_F::Precision;
template<int dim, typename Generator, typename Distribution>
void BM_NoiseVector(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<dim, Generator, Distribution>();
	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
};

BENCHMARK_TEMPLATE(BM_NoiseVector, 3, std::mt19937_64, std::normal_distribution<Precision>);
#ifdef USE_BOOST_RANDOM
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, boost::mt19937_64, std::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, std::mt19937_64, boost::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, boost::mt19937_64, boost::normal_distribution<Precision>);
#ifdef USE_PCG_RANDOM
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, pcg64_k32, boost::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, pcg64_k32_fast, boost::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, pcg32_k1024, boost::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, pcg64_k1024, boost::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, pcg32_k1024_fast, boost::normal_distribution<Precision>);
	BENCHMARK_TEMPLATE(BM_NoiseVector, 3, pcg64_k1024_fast, boost::normal_distribution<Precision>);
#endif
#endif

BENCHMARK_TEMPLATE(BM_NoiseVector, 6, std::mt19937_64, std::normal_distribution<Precision>);
#ifdef USE_BOOST_RANDOM
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, boost::mt19937_64, std::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, std::mt19937_64, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, boost::mt19937_64, boost::normal_distribution<Precision>);
#ifdef USE_PCG_RANDOM
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, pcg64_k32, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, pcg64_k32_fast, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, pcg32_k1024, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, pcg64_k1024, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, pcg32_k1024_fast, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_NoiseVector, 6, pcg64_k1024_fast, boost::normal_distribution<Precision>);
#endif
#endif
}


