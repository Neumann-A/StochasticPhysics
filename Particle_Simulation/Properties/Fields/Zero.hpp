
#ifndef INC_ZeroFieldProperties_H
#define INC_ZeroFieldProperties_H
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
    struct Zero :General<prec> {
        using ThisClass = Zero<prec>;
        static const IField _TypeOfField{ Properties::IField::Field_Zero };
    };

    template<typename Precision, typename Archive>
    void serialize(Zero<Precision>& val, Archive& ar)
    {
    }

}
#endif