
#ifndef INC_LissajousFieldProperties_H
#define INC_LissajousFieldProperties_H
///---------------------------------------------------------------------------------------------------
#pragma once


#include <vector>

#include "Fields/FieldList.h"
#include "Field.hpp"

namespace Properties::Fields
{
    template<typename prec>
    struct Lissajous :General<prec> {

        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;

        static const IField     TypeOfField{ Properties::IField::Field_Lissajous };

        Vec3D                   OffsetField{ Vec3D::Zero() };
        Vec3D                   Amplitude{ Vec3D::Zero() };
        Vec3D                   Frequencies{ 0 };
        Vec3D                   Phases{ 0 };
        
        using ThisClass = Lissajous<prec>;
    };
   
    template<typename Precision, typename Archive>
    void serialize(Lissajous<Precision>& val, Archive& ar)
    {
        ar(Archives::createNamedValue("Offset_Field",val.OffsetField));
        ar(Archives::createNamedValue("Amplitude",val.Amplitude));
        ar(Archives::createNamedValue("Frequencies",val.Frequencies));
        ar(Archives::createNamedValue("Phases",val.Phases));
    }

}
#endif
