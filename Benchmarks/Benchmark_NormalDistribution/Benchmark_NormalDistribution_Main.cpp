///-------------------------------------------------------------------------------------------------
// file:	Benchmark_NormalDistribution_Main.cpp
//
// summary:	Implements the benchmark normal distribution main class
///-------------------------------------------------------------------------------------------------


#ifdef _MSC_VER
#pragma comment (lib, "shlwapi")
#endif

//Google Benchmark!
#include <benchmark/benchmark.h>

#include <random>
#include <array>
#include <algorithm>
#include <functional>
#include <pcg_random.hpp>
#include <type_traits>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#ifdef BENCH_NOISEFIELD

#include "../../Particle_Simulation/SDEFramework/NoiseField.h"

using Noisefield1D = NoiseField<double, 1, pcg64_k1024_fast>;
using Noisefield3D = NoiseField<double, 3, pcg64_k1024_fast>;
using Noisefield6D = NoiseField<double, 6, pcg64_k1024_fast>;

template<class NoiseField>
static void BM_NoiseField(benchmark::State& state)
{
	auto Field = NoiseField(1'000'000,1.0);
	for(auto _ : state)
	{
		benchmark::DoNotOptimize(Field.getField());
	};
};
BENCHMARK_TEMPLATE(BM_NoiseField, Noisefield1D);
BENCHMARK_TEMPLATE(BM_NoiseField, Noisefield3D);
BENCHMARK_TEMPLATE(BM_NoiseField, Noisefield6D);

#endif

template<typename T>
struct is_pcgrandom : public std::false_type {};
template<>
struct is_pcgrandom<pcg64_k1024> : public std::true_type {};
template<>
struct is_pcgrandom<pcg64_k1024_fast> : public std::true_type {};
template<>
struct is_pcgrandom<pcg32_k1024> : public std::true_type {};
template<>
struct is_pcgrandom<pcg32_k1024_fast> : public std::true_type {};
template<>
struct is_pcgrandom<pcg64_k32> : public std::true_type {};
template<>
struct is_pcgrandom<pcg64_k32_fast> : public std::true_type {};

template<typename Rndgen, typename Dist>
class NormalDistFixture : public ::benchmark::Fixture
{

public:
	using RandomGenerator = Rndgen;
	using Distribution = Dist;

	NormalDistFixture() = default;

};
template<typename RandomGenerator>
static RandomGenerator createGenerator()
{
	if constexpr (is_pcgrandom<RandomGenerator>::value)
	{
		// Seed with a real random value, if available
		pcg_extras::seed_seq_from<std::random_device> seq;
		return RandomGenerator{ seq };
	}
	else
	{
		std::random_device rd;
		std::array<std::random_device::result_type, RandomGenerator::state_size> seed_data;
		std::generate(seed_data.begin(), seed_data.end(), [&]() {return rd(); });
		std::seed_seq seq(std::begin(seed_data), std::end(seed_data));
		return RandomGenerator{ seq };
	}
};



template<class RandomGenerator>
static void BM_Generator(benchmark::State& state)
{

	auto gen = createGenerator<RandomGenerator>();
	gen.discard(1'000'000);
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(gen());
	};
};

template<class RandomGenerator, class Distribution>
static void BM_Distribution(benchmark::State& state)
{
	
	auto gen = createGenerator<RandomGenerator>();
	//std::cout << "Sizeof generator type: " << sizeof(RandomGenerator) << "\n";
	//std::cout << "Sizeof generator: " << sizeof(gen) << std::endl;
	gen.discard(1'000'000);
	Distribution dist(0.0,1.0);

	//volatile Precision tmp{ 0.0 };

	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(dist(gen));
	};
};

using Precision = double;

BENCHMARK_TEMPLATE(BM_Generator, std::mt19937_64);
BENCHMARK_TEMPLATE(BM_Generator, boost::mt19937_64);
BENCHMARK_TEMPLATE(BM_Generator, pcg64_k1024);
BENCHMARK_TEMPLATE(BM_Generator, pcg64_k1024_fast);
BENCHMARK_TEMPLATE(BM_Generator, pcg32_k1024);
BENCHMARK_TEMPLATE(BM_Generator, pcg32_k1024_fast);
BENCHMARK_TEMPLATE(BM_Generator, pcg64_k32);
BENCHMARK_TEMPLATE(BM_Generator, pcg64_k32_fast);

BENCHMARK_TEMPLATE(BM_Distribution, std::mt19937_64, std::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, boost::mt19937_64, std::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg64_k1024, std::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg64_k1024_fast, std::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, std::mt19937, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, std::mt19937_64, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, boost::mt19937_64, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg64_k1024, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg64_k1024_fast, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg32_k1024, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg32_k1024_fast, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, pcg64_k32, boost::normal_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, boost::mt19937_64, std::uniform_real_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, boost::mt19937_64, boost::random::uniform_real_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, std::mt19937_64, std::uniform_real_distribution<Precision>);
BENCHMARK_TEMPLATE(BM_Distribution, std::mt19937_64, boost::random::uniform_real_distribution<Precision>);

//BENCHMARK_F(NormalDistFixture<std::mt19937_64,std::normal_distribution<Precision>>, NormalDistSpeedTest)(::benchmark::State& state)
//{
//	while (state.KeepRunning())
//	{
//
//	};
//};


int main(int argc, char** argv)
{
	//for (auto& test_input : { /* ... */ })
	//	benchmark::RegisterBenchmark(test_input.name(), BM_test, test_input);
	//Could also use the Makro: BENCHMARK_MAIN()                   
	::benchmark::Initialize(&argc, argv);
	::benchmark::RunSpecifiedBenchmarks();
	return 0;
}