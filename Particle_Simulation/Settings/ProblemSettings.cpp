///-------------------------------------------------------------------------------------------------
// file:	ProblemSettings.cpp
//
// summary:	Implements the problem settings class
///-------------------------------------------------------------------------------------------------
#include "ProblemSettings.h"

namespace Settings
{
    // const std::map<IProblem, std::string> IProblemMap = { { { IProblem::Problem_undefined,"undefined" },
    //                                                 { IProblem::Problem_BrownAndNeel,"BrownAndNeel" },
    //                                                 { IProblem::Problem_BrownAndNeelEulerSpherical,"BrownAndNeelEulerSpherical" },
    //                                                 { IProblem::Problem_Neel,"Neel" },
    //                                                 { IProblem::Problem_NeelSpherical,"NeelSpherical" },
    //                                                 { IProblem::Problem_NeelQuaternion,"NeelQuaternion" } } };
    std::string to_string(const IProblem& field)
    {
        return std::string{IProblemMap[field]};
    }

    template<>
    IProblem from_string<IProblem>(const std::string& str)
    {
        return IProblemMap[str];
    }

    IProblem from_string(std::string_view str, IProblem& value)
    {
        return (value = IProblemMap[str]);
    }
}

