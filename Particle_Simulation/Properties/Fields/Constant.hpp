
#ifndef INC_ConstantFieldProperties_H
#define INC_ConstantFieldProperties_H
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
    struct Constant :General<prec> {
        using ThisClass = Constant<prec>;
    };

    template<typename Precision, typename Archive>
    void serialize(Constant<Precision>& val, Archive& ar)
    {}

}
#endif