///-------------------------------------------------------------------------------------------------
// file:	BM_NeelProblem_F.cpp
//
// summary:	Implements the bm neel problem f class
///-------------------------------------------------------------------------------------------------
#include "BM_NeelProblem_F.h"

BENCHMARK_F(BM_NeelProblem_F, DeterministicVectorTest)(::benchmark::State& state)
{
	Vec3D y1, x1;
	y1 << 0.658, -0.456, 0.236;
	y1.normalize();

	x1 << 0.155, .07, -2.1;

	Vec3D Res;

	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = mProblem.getDeterministicVector(y1,x1).eval());
	};
}

BENCHMARK_F(BM_NeelProblem_F, StochasticMatrixTest)(::benchmark::State& state)
{
	Vec3D y1;
	y1 << 0.658, -0.456, 0.236;
	y1.normalize();
	Problem::StochasticMatrixType Res;
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = mProblem.getStochasticMatrix(y1).eval());
	};
}

BENCHMARK_F(BM_NeelProblem_F, DriftVectorTest)(::benchmark::State& state)
{
	Vec3D y1;
	y1 << 0.658, -0.456, 0.236;
	y1.normalize();
	Vec3D Res;
	while (state.KeepRunning())
	{
		benchmark::DoNotOptimize(Res = mProblem.getDrift(y1).eval());
	};
}

