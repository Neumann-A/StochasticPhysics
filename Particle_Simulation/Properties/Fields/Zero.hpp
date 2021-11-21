
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

        typedef Eigen::Matrix<prec, 3, 1>   Vec3D;
        static const IField TypeOfField{ Properties::IField::Field_Zero };

        Vec3D                   OffsetField{ Vec3D::Zero() };

    };

    template<typename Precision, typename Archive>
    void serialize(Zero<Precision>& val, Archive& ar) {
        ar(Archives::createNamedValue("Offset_Field",val.OffsetField));
    }

}
#endif
