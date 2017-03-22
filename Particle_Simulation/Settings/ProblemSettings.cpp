///-------------------------------------------------------------------------------------------------
// file:	ProblemSettings.cpp
//
// summary:	Implements the problem settings class
///-------------------------------------------------------------------------------------------------
#include "ProblemSettings.h"

namespace Settings
{
	std::string to_string(const IProblem& field)
	{
		return IProblemMap.at(field);
	};

	template<>
	IProblem from_string<IProblem>(const std::string &String)
	{
		for (auto it : IProblemMap)
			if (it.second == String)
				return it.first;

		throw std::runtime_error{ std::string{ "SolverSettings: Type of Solver unknown! " } +String };
	};
}




//namespace Archives
//{
//	///-------------------------------------------------------------------------------------------------
//	/// <summary>	A load constructor interface for archives.
//	/// 			Common helper class to construct not default constructable objects
//	/// 			Use a specialization of this class for not default constructable objects. </summary>
//	///
//	/// <typeparam name="ToConstruct">	Type to construct. </typeparam>
//	///-------------------------------------------------------------------------------------------------
//	template<typename ToConstruct>
//	class LoadConstructor
//	{
//		using type = ToConstruct;
//
//		static_assert(std::is_constructible<type, void>::value, "Type is not constructable without arguments. Archives need specialization of LoadConstructor ");
//
//		template <typename Archive>
//		static inline type construct(InputArchive<Archive>& ar)
//		{
//			type ConstructedType{};
//			ar(ConstructedType);
//			return ConstructedType;
//		};
//
//		template <typename Archive>
//		static inline type constructWithName(InputArchive<Archive>& ar, char const * const name)
//		{
//			type ConstructedType{};
//			ar(Basic::createNamedValue(name, ConstructedType));
//			return ConstructedType;
//		};
//
//		template <typename Archive>
//		static inline type constructWithName(InputArchive<Archive>& ar, const std::string& name)
//		{
//			type ConstructedType{};
//			ar(Basic::createNamedValue(name, ConstructedType));
//			return ConstructedType;
//		};
//
//		template <typename Archive>
//		static inline type constructWithName(const std::string& name, InputArchive<Archive>& ar)
//		{
//			return constructWithName(ar, name);
//		};
//
//		template <typename Archive>
//		static inline type constructWithName(char const * const name, InputArchive<Archive>& ar)
//		{
//			return constructWithName(ar, name);
//		};
//
//	};
//}
