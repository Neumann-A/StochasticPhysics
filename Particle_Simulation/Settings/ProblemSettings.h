///---------------------------------------------------------------------------------------------------
// file:		ProblemSettings.h
//
// summary: 	Declares the problem settings class
//
// Copyright (c) 2016 Alexander Neumann.
//
// author: Alexander Neumann
// date: 24.06.2016

#ifndef INC_ProblemSettings_H
#define INC_ProblemSettings_H
///---------------------------------------------------------------------------------------------------
#pragma once

#include <memory>

#include "Archive/NamedValue.h"
#include "Archive/LoadConstructor.h"
#include "Archive/OutputArchive.h"
#include "Archive/InputArchive.h"

namespace Settings
{
	//Forward declrations of problem settings:
	template<typename prec>
	class BrownAndNeelProblemSettings;
	template<typename prec>
	class NeelProblemSettings;

	/// <summary>	Values that represent differen Problems to simulate. </summary>
	enum class IProblem { Problem_undefined, Problem_BrownAndNeel, Problem_Neel};

#ifdef _MSC_VER
#pragma warning (push)
#pragma warning ( disable : 4592) // Disable stupid VS Debug message
#endif
	/// <summary>	Map used to change the IProblem enum to a string and vice versa. </summary>
	const std::map<IProblem, std::string> IProblemMap{ { { IProblem::Problem_undefined,"undefined" },{ IProblem::Problem_BrownAndNeel,"BrownAndNeel" },{ IProblem::Problem_Neel,"Neel" } } };
#ifdef _MSC_VER
#pragma warning (pop)
#endif

	///-------------------------------------------------------------------------------------------------
	/// <summary>	Gets the enum IProblem from a string. </summary>
	///
	/// <param name="AnisoString">	The string to transform </param>
	///
	/// <returns>	An Enum representing the string  </returns>
	///-------------------------------------------------------------------------------------------------
	std::string to_string(const IProblem&);
	template<typename T>
	T from_string(const std::string&);
	template<>
	IProblem from_string<IProblem>(const std::string&);

	template<typename prec>
	class IProblemSettings
	{
	private:
		typedef IProblemSettings<prec>	ThisClass;

	protected:
		//virtual void WriteValuesToConfigFileDerived(ConfigFile &file, const std::string &section) const = 0;
		//virtual void InsertIntoMATLABarrayDerived(mxArray& mat) const = 0;
		IProblemSettings() = default;
		//virtual ThisClass* getNewProblemSettings(const ConfigFile &file, const std::string &section) = 0;

	public:
		virtual IProblem getProblemType() const noexcept = 0;
		virtual ~IProblemSettings() = default;
		virtual std::unique_ptr<ThisClass> clone() const noexcept = 0;

		static inline std::string getSectionName() { return std::string{ "Problem_Settings" }; };
		static inline std::string getProblemString() { return std::string{ "Problem_type" }; };

		
		template< typename Archive>
		std::enable_if_t<std::is_base_of<::Archives::OutputArchive<std::decay_t<Archive>>, std::decay_t<Archive>>::value> serialize(Archive& ar)
		{
			const auto Problem = this->getProblemType();
			const std::string problemstr = ::Settings::to_string(this->getProblemType());
			ar(Archives::createNamedValue(ThisClass::getProblemString(),problemstr));
			
			switch (Problem)
			{
			case Settings::IProblem::Problem_BrownAndNeel:
			{
				dynamic_cast<Settings::BrownAndNeelProblemSettings<prec>&>(*this).serialize(ar);
				break;
			}
			case Settings::IProblem::Problem_Neel:
			{
				dynamic_cast<Settings::NeelProblemSettings<prec>&>(*this).serialize(ar);
				break;
			}
			default:
			{
				break;
			}
			}

		}
	};

	template<typename prec>
	class BrownAndNeelProblemSettings : public IProblemSettings<prec>
	{
	private:
		bool _UseSimpleModel{ false };
		typedef BrownAndNeelProblemSettings<prec>	ThisClass;
		typedef IProblemSettings<prec>				ProblemInterface;

	protected:

	public:
		IProblem getProblemType() const noexcept override final
		{
			return IProblem::Problem_BrownAndNeel;
		};

		std::unique_ptr<ProblemInterface> clone() const noexcept override final
		{
			return std::make_unique<ThisClass>(*this);
		}

		BrownAndNeelProblemSettings(bool usesimple) : _UseSimpleModel(usesimple) {};
		BrownAndNeelProblemSettings() = default;

		static std::string getSectionName() { return std::string{ "BrownAndNeel_Problem_Settings" }; };

		template<typename Archive>
		void serialize(Archive &ar)
		{
			ar(Archives::createNamedValue("Use_simple_model", _UseSimpleModel));
		}

		//static ThisClass createObjectFromConfigFile(const ConfigFile &file, const std::string &section)
		//{
		//	bool useSimple{ file.getBooleanOfKeyInSection(section, SMEMBERNAME(_UseSimpleModel)) };
		//	return ThisClass{ useSimple };
		//};

		// Access the UseSimpleModel
		bool getUseSimpleModel(void) const noexcept { return(_UseSimpleModel); }
		void setUseSimpleModel(bool useSimpleModel)	noexcept { _UseSimpleModel = useSimpleModel; }
	};

	template<typename prec>
	class NeelProblemSettings : public IProblemSettings<prec>
	{

	private:
		typedef NeelProblemSettings<prec>			ThisClass;
		typedef IProblemSettings<prec>				ProblemInterface;

	protected:

	public:
		std::unique_ptr<ProblemInterface> clone() const noexcept override final
		{
			return std::make_unique<ThisClass>(*this);
		}

		static std::string getSectionName() { return std::string{ "Neel_Problem_Settings" }; };

		template<typename Archive>
		void serialize(Archive &) 
		{ /*Nothing to do in this method yet*/ }

	
		IProblem getProblemType() const noexcept override final
		{
			return IProblem::Problem_Neel;
		};

	};

}

namespace Archives
{
	template<typename ToConstruct>
	class LoadConstructor;

	template<typename prec>
	class LoadConstructor<typename Settings::template IProblemSettings<prec>>
	{
	private:
		using ToConstruct = typename Settings::template IProblemSettings<prec>;
	public:
		template <typename Archive>
		static inline decltype(auto) construct(InputArchive<Archive>& ar)
		{
			auto& arch = static_cast<typename InputArchive<Archive>::ArchiveType&>(ar);
			
			std::string ProblemType;
			arch(Archives::createNamedValue(ToConstruct::getSectionName(),Archives::createNamedValue(ToConstruct::getProblemString(), ProblemType)));
			const auto type{ Settings::from_string<Settings::IProblem>(ProblemType) };
			std::unique_ptr<ToConstruct> tmp{ nullptr };
			
			switch (type)
			{
			case Settings::IProblem::Problem_BrownAndNeel:
			{				
				Settings::BrownAndNeelProblemSettings<prec> set;
				arch(Archives::createNamedValue(ToConstruct::getSectionName(),set));
				tmp = std::make_unique<Settings::BrownAndNeelProblemSettings<prec>>(set);
				break;
			}
			case Settings::IProblem::Problem_Neel:
			{
				Settings::NeelProblemSettings<prec> set;
				arch(Archives::createNamedValue(ToConstruct::getSectionName(), set));
				tmp = std::make_unique<Settings::NeelProblemSettings<prec>>(set);
				break;
			}
			default:
				tmp = nullptr;
				break;
			}
			return tmp;
		};
	
	};
}


#endif	// INC_ProblemSettings_H
// end of ProblemSettings.h
///---------------------------------------------------------------------------------------------------
