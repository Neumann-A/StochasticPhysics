// stdafx.h: Includedatei für Standardsystem-Includedateien
// oder häufig verwendete projektspezifische Includedateien,
// die nur in unregelmäßigen Abständen geändert werden.
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

// TODO: Hier auf zusätzliche Header, die das Programm erfordert, verweisen.
