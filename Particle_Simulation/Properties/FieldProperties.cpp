///-------------------------------------------------------------------------------------------------
// file:	FieldProperties.cpp
//
// summary:	Implements details of field Properties
///-------------------------------------------------------------------------------------------------

#include "FieldProperties.h"

namespace Properties
{
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