// stdafx.h: Includedatei f�r Standardsystem-Includedateien
// oder h�ufig verwendete projektspezifische Includedateien,
// die nur in unregelm��igen Abst�nden ge�ndert werden.
//

#pragma once

#include <benchmark/benchmark.h>
#include <Eigen/Core>
#include <random>
#include <pcg_random.hpp>


#ifdef _MSC_VER
#pragma comment (lib, "shlwapi")
#endif

#ifdef __clang__
#undef _MSC_VER //Silly hack to get boost PP running with clang-cl
#endif
#include <boost/random.hpp>

// TODO: Hier auf zus�tzliche Header, die das Programm erfordert, verweisen.
