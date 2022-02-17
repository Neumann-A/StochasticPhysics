
#ifndef INC_SincFieldProperties_H
#define INC_SincFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <vector>

#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Sinc :General<prec> {

        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;
        static const IField     TypeOfField{ Properties::IField::Field_Sinc };

        Vec3D                   OffsetField{ Vec3D::Zero() };
        Vec3D                   Amplitude{ Vec3D::Zero() };
        prec                    Periode{ 0 };
        prec                    TimeOffset{ 0 };

        using ThisClass = Sinc<prec>;
    };

    template<typename Precision, typename Archive>
    void serialize(Sinc<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Offset_Field",val.OffsetField));
        ar(Archives::createNamedValue("Amplitude",val.Amplitude));
        ar(Archives::createNamedValue("Periode",val.Periode));
        ar(Archives::createNamedValue("TimeOffset",val.TimeOffset));
    }

}
#endif
