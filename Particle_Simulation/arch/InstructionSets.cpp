#include "InstructionSets.hpp"

#include <map>
#include <istream>
#if defined(_MSC_VER) 
#include <intrin0.h>
#include <isa_availability.h>
extern "C" long __isa_enabled;
#endif

// Maybe use https://docs.microsoft.com/de-de/cpp/intrinsics/cpuid-cpuidex?view=vs-2019

namespace MyCEL::SystemInfo {



    const std::map<InstructionSet, std::string_view>& getInstructionSetMap() {

        static const std::map<InstructionSet, std::string_view> InstructionSetMap{ { { InstructionSet::NONE,"NONE" },
                                                        { InstructionSet::AVX,"AVX" },
                                                        { InstructionSet::AVX2,"AVX2" },
                                                        { InstructionSet::AVX512,"AVX512" }} };
        return InstructionSetMap;
    }

	const std::string_view& to_string(const InstructionSet& set)
	{         
		return getInstructionSetMap().at(set);
	};

	template<>
	std::optional<InstructionSet> from_string<InstructionSet>(std::string_view str)
	{
		for (auto it : getInstructionSetMap())
			if (it.second == str)
				return it.first;

        return std::nullopt;
	};

    InstructionSet getCPUInstructionSet()
    {
        #if defined(__GNUC__) || ( defined (__clang__) && !defined(_MSC_VER) )
            __builtin_cpu_init();
            if(__builtin_cpu_supports("avx512f")) return InstructionSet::AVX512;
            else if(__builtin_cpu_supports("avx2")) return InstructionSet::AVX2;
            else if(__builtin_cpu_supports("avx")) return InstructionSet::AVX;
            
        #elif defined(_MSC_VER)
            if(_bittest(&__isa_enabled, __ISA_AVAILABLE_AVX512)) return InstructionSet::AVX512;
            else if(_bittest(&__isa_enabled, __ISA_AVAILABLE_AVX2)) return InstructionSet::AVX2;
            else if(_bittest(&__isa_enabled, __ISA_AVAILABLE_AVX)) return InstructionSet::AVX;
        #endif
            else
            {
                return InstructionSet::NONE;
            }
    }


}
