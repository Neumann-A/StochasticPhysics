///-------------------------------------------------------------------------------------------------
// file:	PropertyProvider.h
//
// summary:	Declares the Interface for Provider classes
///-------------------------------------------------------------------------------------------------

#pragma once

#include <MyCEL/basics/BasicMacros.h>

#include "Settings/ParticleSimulationParameters.h"

///-------------------------------------------------------------------------------------------------
/// <summary>	Values that represent a Property Class. </summary>
///
/// <remarks>	Alexander Neumann, 05.06.2016. </remarks>
///-------------------------------------------------------------------------------------------------

enum class IProperty { ParticleProperties, AtomisticProperties};

///-------------------------------------------------------------------------------------------------
/// <signature>	Provider. </signature>
///
/// <summary>	Namespace for the Provider classes. </summary>
///-------------------------------------------------------------------------------------------------

namespace Provider
{
	///-------------------------------------------------------------------------------------------------
	/// <summary>	General Abstract Class for Providers  </summary>
	///
	/// <remarks>	Alexander Neumann, 05.06.2016. </remarks>
	///
	/// <seealso cref="T:IMATLABFileWriteable"/>
	/// <seealso cref="T:IConfigFileAll{IGeneralProvider{prec}}"/>
	///
	/// <typeparam name="prec">	Floating point precission. </typeparam>
	///-------------------------------------------------------------------------------------------------
	template <typename prec>
	class IGeneralProvider 
	{
		MY_INTERFACE(IGeneralProvider<prec>)
		ALLOW_DEFAULT_COPY_AND_ASSIGN(IGeneralProvider<prec>)
	private:
		typedef IGeneralProvider<prec>		ThisClass;
	protected:
	public:
		typedef prec						Precision;
		//using IConfigFileAll<ThisClass>::WriteValuesToConfigFile;
		//using IConfigFileAll<ThisClass>::GetValuesFromConfigFile;
		//using IConfigFileAll<ThisClass>::createObjectFromConfigFile;
		//using IMATLABFileWriteable::WriteValuesToMATLABFile;

		///-------------------------------------------------------------------------------------------------
		/// <summary>	Gets provided object. </summary>
		///
		/// <remarks>	Alexander Neumann, 05.06.2016. </remarks>
		///
		/// <typeparam name="T">	Generic type of Object Provided. </typeparam>
		///
		/// <returns>	The provided object. </returns>
		///-------------------------------------------------------------------------------------------------
		virtual Parameters::ParticleSimulationParameters<prec>	getProvidedObject() = 0;
		virtual unsigned long long getNumberOfNecessarySimulations() const noexcept = 0;

	};
}
