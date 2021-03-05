
#ifndef INC_RectangularFieldProperties_H
#define INC_RectangularFieldProperties_H
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
    struct Rectangular :General<prec>{

        std::vector<prec> _Periodes{ 0 };
        std::vector<prec> _PhasesTimeOffsets{ 0 };
        prec _Tau{ 0.0 };
        bool _alternating{ false };

        using ThisClass = Rectangular<prec>;
    };

    template<typename Precision, typename Archive>
    void serialize(Rectangular<Precision>& val, Archive& ar)
    {
        serializeVector(ar, "Number_of_Periodes", "Periode_", val._Periodes);
        serializeVector(ar, "Number_of_Phases", "Phase_", val._PhasesTimeOffsets);
        ar(Archives::createNamedValue("Tau", val._Tau));
        ar(Archives::createNamedValue("Alternating", val._alternating));
    }
}
#endif