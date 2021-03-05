///-------------------------------------------------------------------------------------------------
// file:	FieldProperties.cpp
//
// summary:	Implements details of field Properties
///-------------------------------------------------------------------------------------------------

#include "FieldProperties.h"

namespace Properties
{
    const std::map<IField, std::string> IFieldMap = { { { IField::Field_undefined,"undefined" },
                                                            { IField::Field_Zero,"none" },
                                                            { IField::Field_Constant,"constant" },
                                                            { IField::Field_Sinusoidal,"sinusoidal" },
                                                            { IField::Field_Lissajous,"lissajous" },
                                                            { IField::Field_Triangular,"triangular"},
                                                            { IField::Field_Rectangular,"rectangular"},
                                                            { IField::Field_Sinc,"sinc"}} };
    std::string to_string(const IField& field)
    {
        return IFieldMap.at(field);
    };

    template<>
    IField from_string<IField>(const std::string &String)
    {
        for (auto it : IFieldMap)
            if (it.second == String)
                return it.first;

        throw std::runtime_error{ std::string{"FieldParameters: Type of Field unknown! "} +String };
    };
}
