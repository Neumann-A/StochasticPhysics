
#ifndef INC_SinusoidalFieldProperties_H
#define INC_SinusoidalFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <vector>

#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Sinusoidal :General<prec> {

        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;
        static const IField         TypeOfField{ Properties::IField::Field_Sinusoidal };

        Vec3D                       OffsetField{ Vec3D::Zero() };
        Vec3D                       Amplitude{ Vec3D::Zero() };
        prec                        Frequency{ 0.0 };
        prec                        Phase{ 0.0 };

        using ThisClass = Sinusoidal<prec>;
    };
    template<typename Precision, typename Archive>
    void serialize(Sinusoidal<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Offset_Field",val.OffsetField));
        ar(Archives::createNamedValue("Amplitude",val.Amplitude));
        ar(Archives::createNamedValue("Frequency",val.Frequency));
        ar(Archives::createNamedValue("Phase",val.Phase));
    }
}
#endif
