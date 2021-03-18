
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
        Vec3D                       Amplitudes{ Vec3D::Zero() };
        prec                        Frequencies{ 0 };
        prec                        PhasesTimeOffsets{ 0 };
        

        using ThisClass = Sinusoidal<prec>;
    };
    template<typename Precision, typename Archive>
    void serialize(Sinusoidal<Precision>& val, Archive& ar)
    {
        typedef Eigen::Matrix<Precision, 3, 1>                                Vec3D;
        using Vec3DList = std::vector<Vec3D>;
        Vec3DList               vec{ val.OffsetField,val.Amplitudes };
        std::vector<Precision>  Freq{ val.Frequencies };
        std::vector<Precision>  Phase{ val.PhasesTimeOffsets };

        serializeVector(ar, "Number_of_Amplitudes", "Amplitude_", vec);
        val.OffsetField = vec.at(0);
        val.Amplitudes = vec.at(1);

        serializeVector(ar, "Number_of_Frequencies", "Frequency_", Freq);
        val.Frequencies = Freq.at(0);

        serializeVector(ar, "Number_of_Phases", "Phase_", Phase);
        val.PhasesTimeOffsets = Phase.at(0);
    }
}
#endif
