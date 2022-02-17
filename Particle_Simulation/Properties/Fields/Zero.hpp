
#ifndef INC_ZeroFieldProperties_H
#define INC_ZeroFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <vector>

#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Zero : General<prec> {
        using ThisClass = Zero<prec>;
        static const IField TypeOfField{ Properties::IField::Field_Zero};
    };

    template<typename Precision, typename Archive>
    void serialize(Zero<Precision>&, Archive&) {}
}
#endif
