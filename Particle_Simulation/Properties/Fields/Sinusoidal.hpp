
#ifndef INC_SinusoidalFieldProperties_H
#define INC_SinusoidalFieldProperties_H
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
    struct Sinusoidal :General<prec> {

        std::vector<prec> _Frequencies{ 0 };
        std::vector<prec> _PhasesTimeOffsets{ 0 };
        

        using ThisClass = Sinusoidal<prec>;
    };
    template<typename Precision, typename Archive>
    void serialize(Sinusoidal<Precision>& val, Archive& ar)
    {
        serializeVector(ar, "Number_of_Frequencies", "Frequency_", val._Frequencies);
        serializeVector(ar, "Number_of_Phases", "Phase_", val._PhasesTimeOffsets);

    }
}
#endif