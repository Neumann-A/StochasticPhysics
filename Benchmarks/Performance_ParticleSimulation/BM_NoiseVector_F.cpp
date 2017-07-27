///-------------------------------------------------------------------------------------------------
// file:	BM_NoiseVector_F.cpp
//
// summary:	Implements the bm noise vector f class
///-------------------------------------------------------------------------------------------------
#include "BM_NoiseVector_F.h"



BENCHMARK_F(BM_NoiseVector_F, boost_mt19937_64_Dist_boost_3D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<3, boost::mt19937_64, boost::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

BENCHMARK_F(BM_NoiseVector_F, boost_mt19937_64_Dist_std_3D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<3, boost::mt19937_64, std::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

BENCHMARK_F(BM_NoiseVector_F, std_mt19937_64_Dist_boost_3D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<3, std::mt19937_64, boost::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

BENCHMARK_F(BM_NoiseVector_F, std_mt19937_64_Dist_std_3D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<3, std::mt19937_64, std::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k1024_Dist_boost_3D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<3, pcg64_k1024, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k1024_fast_Dist_boost_3D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<3, pcg64_k1024_fast, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg32_k1024_Dist_boost_3D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<3, pcg32_k1024, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg32_k1024_fast_Dist_boost_3D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<3, pcg32_k1024_fast, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k32_Dist_boost_3D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<3, pcg64_k32, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k32_fast_Dist_boost_3D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<3, pcg64_k32_fast, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}

///-------------------------------------------------------------------------------------------------
// End of 3D
///-------------------------------------------------------------------------------------------------

BENCHMARK_F(BM_NoiseVector_F, boost_mt19937_64_Dist_boost_6D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<6, boost::mt19937_64, boost::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

BENCHMARK_F(BM_NoiseVector_F, boost_mt19937_64_Dist_std_6D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<6, boost::mt19937_64, std::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

BENCHMARK_F(BM_NoiseVector_F, std_mt19937_64_Dist_boost_6D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<6, std::mt19937_64, boost::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

BENCHMARK_F(BM_NoiseVector_F, std_mt19937_64_Dist_std_6D)(::benchmark::State& state)
{
	auto Field = BM_NoiseVector_F::generateField<6, std::mt19937_64, std::normal_distribution<Precision>>();

	auto Res = Field.getField().eval();
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = Field.getField().eval());
	};
}

//BENCHMARK_F(BM_NoiseVector_F, pcg64_k1024_Dist_boost_6D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<6, pcg64_k1024, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k1024_fast_Dist_boost_6D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<6, pcg64_k1024_fast, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg32_k1024_Dist_boost_6D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<6, pcg32_k1024, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg32_k1024_fast_Dist_boost_6D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<6, pcg32_k1024_fast, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k32_Dist_boost_6D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<6, pcg64_k32, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}
//
//BENCHMARK_F(BM_NoiseVector_F, pcg64_k32_fast_Dist_boost_6D)(::benchmark::State& state)
//{
//	auto Field = BM_NoiseVector_F::generateField<6, pcg64_k32_fast, boost::normal_distribution<Precision>>();
//
//	auto Res = Field.getField().eval();
//	while (state.KeepRunning())
//	{
//		benchmark::DoNotOptimize(Res = Field.getField().eval());
//	};
//}