///-------------------------------------------------------------------------------------------------
// file:	Benchmark_NormalDistribution_Main.cpp
//
// summary:	Implements the benchmark normal distribution main class
///-------------------------------------------------------------------------------------------------

//Google Benchmark!
#include <benchmark/benchmark.h>

#include <random>
#include <array>
#include <algorithm>
#include <functional>
#include <pcg_random.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <type_traits>

#ifdef _MSC_VER
#pragma comment (lib, "shlwapi")
#endif
//#ifdef NDEBUG
//#pragma comment (lib, "Archives")
//#pragma comment (lib, "Basic_Library")
//#else
//#pragma comment (lib, "Archives_D")
//#pragma comment (lib, "Basic_Library_D")
//#endif

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

template<class RandomGenerator, class Distribution>
static void BM_Distribution(benchmark::State& state)
{
	auto gen = createGenerator<RandomGenerator>();
	gen.discard(1'000'000);
	Distribution dist(0.0,1.0);

	//volatile Precision tmp{ 0.0 };

	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(dist(gen));
	};
};

using Precision = double;

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
BENCHMARK_TEMPLATE(BM_Distribution, pcg64_k32_fast, boost::normal_distribution<Precision>);

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