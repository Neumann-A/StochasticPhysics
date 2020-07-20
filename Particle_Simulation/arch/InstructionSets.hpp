#pragma once

#include <map>
#include <string_view>
#include <optional>
#include <iostream>

namespace MyCEL::SystemInfo {
    
    enum class InstructionSet {NONE, AVX, AVX2, AVX512};

    extern const std::map<InstructionSet, std::string_view>& getInstructionSetMap();

/// VVV Belongs somewhere more general 
    template<typename C>
    std::optional<typename C::value_type::first_type> from_string(const C& con, std::string_view view)
    {
        auto res = std::find_if(con.begin(), con.end(), [&view](const typename C::value_type elem) {return elem.second == view; });
        if (res != con.end())
            return res->first;
        else
            return std::nullopt;
    };
    template<typename C>
    std::string_view to_string_view(C& con, typename C::value_type::first_type type)
    {
        return con.find(type)->second;
    }
/// ^^^^  Belongs somewhere more general 

	template<typename T>
	std::optional<T> from_string(std::string_view);

	template<>
	std::optional<InstructionSet> from_string<InstructionSet>(std::string_view);

	const std::string_view& to_string(const InstructionSet& field);

    InstructionSet getCPUInstructionSet();

    inline std::istream& operator>>(std::istream& in, MyCEL::SystemInfo::InstructionSet& set)
    {
        std::string token;
        in >> token;

        const auto opt_set = MyCEL::SystemInfo::from_string<MyCEL::SystemInfo::InstructionSet>(token);

        if (opt_set)
            set = *opt_set;
        else
            in.setstate(std::ios_base::failbit);
        return in;
    }
    inline std::ostream& operator<<(std::ostream& os,const MyCEL::SystemInfo::InstructionSet& set)
    {
        os << to_string(set);
        return os;
    }
}


//inline std::istream& operator>>(std::istream& in, MyCEL::SystemInfo::InstructionSet& set)
//{
//    std::string token;
//    in >> token;
//
//    const auto opt_set = MyCEL::SystemInfo::from_string<MyCEL::SystemInfo::InstructionSet>(token);
//
//    if (opt_set)
//        set = *opt_set;
//    else
//        in.setstate(std::ios_base::failbit);
//    return in;
//}