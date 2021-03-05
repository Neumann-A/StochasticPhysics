
#ifndef INC_SincFieldProperties_H
#define INC_SincFieldProperties_H
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
    struct Sinc :General<prec> {

        std::vector<prec> _Periodes{ 0 };
        std::vector<prec> _PhasesTimeOffsets{ 0 };

        using ThisClass = Sinc<prec>;
    };

    template<typename Precision, typename Archive>
    void serialize(Sinc<Precision>& val, Archive& ar)
    {
        serializeVector(ar, "Number_of_Periodes", "Periode_", val._Periodes);
        serializeVector(ar, "Number_of_Phases", "Phase_", val._PhasesTimeOffsets);

    }

}
#endif