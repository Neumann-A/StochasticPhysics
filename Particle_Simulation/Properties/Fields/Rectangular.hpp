
#ifndef INC_RectangularFieldProperties_H
#define INC_RectangularFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <vector>

#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Rectangular : General<prec>{

        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;
        static const IField          TypeOfField{ Properties::IField::Field_Rectangular };

        Vec3D                        OffsetField{ Vec3D::Zero() };
        Vec3D                        Amplitude{ Vec3D::Zero() };
        prec                         Periode{ 0 };
        prec                         TimeOffset{ 0 };
        prec                         Tau{ 0.0 };
        bool                         Alternating{ false };

        using ThisClass = Rectangular<prec>;
    };

    template<typename Precision, typename Archive>
    void serialize(Rectangular<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Offset_Field",val.OffsetField));
        ar(Archives::createNamedValue("Amplitude",val.Amplitude));
        ar(Archives::createNamedValue("Tau",val.Tau));
        ar(Archives::createNamedValue("TimeOffset",val.TimeOffset));
        ar(Archives::createNamedValue("Periode",val.Periode));
        ar(Archives::createNamedValue("Alternating", val.Alternating));
    }
}
#endif
