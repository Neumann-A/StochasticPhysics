///-------------------------------------------------------------------------------------------------
// file:	Fields\FieldList.cpp
//
// summary:	Implements the Field list class
///-------------------------------------------------------------------------------------------------
#include "FieldList.h"
#include <string>

namespace Properties
{
    std::string to_string(const IField& field)
    {
        return std::string{IFieldMap[field]};
    }

    template<>
    IField from_string<IField>(const std::string& FieldString)
    {
        return IFieldMap[FieldString];
    }

    IField from_string(std::string_view FieldString, IField& value)
    {
        return (value = IFieldMap[FieldString]);
    }
}
