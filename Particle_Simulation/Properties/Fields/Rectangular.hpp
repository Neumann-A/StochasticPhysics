
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
    struct Rectangular :General<prec>{

        typedef Eigen::Matrix<prec, 3, 1>                                Vec3D;
        static const IField          TypeOfField{ Properties::IField::Field_Rectangular };

        Vec3D                        OffsetField{ Vec3D::Zero() };
        Vec3D                        Amplitudes{ Vec3D::Zero() };
        prec                         Periodes{ 0 };
        prec                         PhasesTimeOffsets{ 0 };
        prec                         Tau{ 0.0 };
        bool                         Alternating{ false };

        using ThisClass = Rectangular<prec>;
    };

    template<typename Precision, typename Archive>
    void serialize(Rectangular<Precision>& val, Archive& ar)
    {
        typedef Eigen::Matrix<Precision, 3, 1>                                Vec3D;
        using Vec3DList = std::vector<Vec3D>;
        Vec3DList               vec{ val.OffsetField,val.Amplitudes };
        std::vector<Precision>  Period{ val.Periodes };
        std::vector<Precision>  Phase{ val.PhasesTimeOffsets };

        serializeVector(ar, "Number_of_Amplitudes", "Amplitude_", vec);
        val.OffsetField = vec.at(0);
        val.Amplitudes = vec.at(1);

        serializeVector(ar, "Number_of_Periodes", "Periode_", Period);
        val.Periodes = Period.at(0);

        serializeVector(ar, "Number_of_Phases", "Phase_", Phase);
        val.PhasesTimeOffsets = Phase.at(0);

        ar(Archives::createNamedValue("Tau", val.Tau));
        ar(Archives::createNamedValue("Alternating", val.Alternating));
    }
}
#endif
