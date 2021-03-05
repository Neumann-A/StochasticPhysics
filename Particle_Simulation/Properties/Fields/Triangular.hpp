
#ifndef INC_TriangularFieldProperties_H
#define INC_TriangularFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <string>
#include <map>

#include <Eigen/Core>
//#include <Eigen/StdVector>

#include <SerAr/Core/NamedValue.h>
#include <SerAr/Core/InputArchive.h>
#include <SerAr/Core/OutputArchive.h>

#include <MyCEL/basics/BasicIncludes.h>
#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Triangular :General<prec> {

        std::vector<prec> _Periodes{ 0 };
        std::vector<prec> _PhasesTimeOffsets{ 0 };

        using ThisClass = Triangular<prec>;
    };
 
    template<typename Precision, typename Archive>
    void serialize(Triangular<Precision>& val, Archive& ar)
    {
        serializeVector(ar, "Number_of_Phases", "Phase_", val._PhasesTimeOffsets);
        serializeVector(ar, "Number_of_Periodes", "Periode_", val._Periodes);
    }
}
#endif