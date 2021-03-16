
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
        Vec3D                   Amplitudes{ Vec3D::Zero() };
        Vec3D                   Frequencies{ 0 };
        Vec3D                   PhasesTimeOffsets{ 0 };
        
        using ThisClass = Lissajous<prec>;
    };
   
    template<typename Precision, typename Archive>
    void serialize(Lissajous<Precision>& val, Archive& ar)
    {
        typedef Eigen::Matrix<Precision, 3, 1>                                Vec3D;
        using Vec3DList = std::vector<Vec3D>;

        Vec3DList vec{ val.OffsetField,val.Amplitudes };

        serializeVector(ar, "Number_of_Amplitudes", "Amplitude_", vec);
        val.OffsetField = vec.at(0);
        val.Amplitudes = vec.at(1);

        serializeVector(ar, "Number_of_Frequencies", "Frequency_", val.Frequencies);
        serializeVector(ar, "Number_of_Phases", "Phase_", val.PhasesTimeOffsets);
    }

}
#endif