
#ifndef INC_ConstantFieldProperties_H
#define INC_ConstantFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <vector>

#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Constant : General<prec> {

        using ThisClass = Constant<prec>;
        

        static const IField TypeOfField{ Properties::IField::Field_Constant };

        typedef Eigen::Matrix<prec, 3, 1>        Vec3D;

        Vec3D                   OffsetField{ Vec3D::Zero() };
    };

    template<typename Precision, typename Archive>
    void serialize(Constant<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Offset_Field",val.OffsetField));
    }

}
#endif
