
#ifndef INC_LissajousFieldProperties_H
#define INC_LissajousFieldProperties_H
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
    struct Lissajous :General<prec> {

        std::vector<prec> _Frequencies{ 0 };
        std::vector<prec> _PhasesTimeOffsets{ 0 };
        
        using ThisClass = Lissajous<prec>;
    };
   
    template<typename Precision, typename Archive>
    void serialize(Lissajous<Precision>& val, Archive& ar)
    {
        serializeVector(ar, "Number_of_Frequencies", "Frequency_", val._Frequencies);
        serializeVector(ar, "Number_of_Phases", "Phase_", val._PhasesTimeOffsets);

    }

}
#endif