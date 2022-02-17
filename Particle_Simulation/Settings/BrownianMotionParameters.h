///-------------------------------------------------------------------------------------------------
// file:	BrownianMotionParameters.h
//
// summary:	Declares the brownian motion parameters class
///-------------------------------------------------------------------------------------------------
#pragma once

#include "BasicParameters.h"
#include "ProblemParameters.h"

// TODO: REFACTOR
namespace Problems
{
    using namespace Parameters;
    template <typename prec>
    class BrownianMotionParameters
        : public virtual IParameters<BrownianMotionParameters<prec>>, public virtual IProblemParameters
    {
    private:
        typedef BrownianMotionParameters<prec>		 ThisClass;

        prec										diffusionCoefficient;
        
        prec										m_a;
        prec										m_b;
        prec										m_c;


    public:

        explicit BrownianMotionParameters(const prec& difcoef, const prec& a, const prec& b, const prec& c) :
            diffusionCoefficient(difcoef), m_a(a), m_b(b), m_c(c)
        {

        }

        static std::string getSectionName() { return std::string{ "BrownianMotionParameters" }; };

        void WriteValuesToConfigFile(ConfigFile &file, const std::string &section) const
        {
            file.addKeyValue(section, SVARNAME(diffusionCoefficient), BasicTools::toStringScientific(diffusionCoefficient));
            file.addKeyValue(section, SVARNAME(m_a), BasicTools::toStringScientific(m_a));
            file.addKeyValue(section, SVARNAME(m_b), BasicTools::toStringScientific(m_b));
            file.addKeyValue(section, SVARNAME(m_c), BasicTools::toStringScientific(m_c));
            
        };
        void WriteValuesToConfigFile(ConfigFile &File) const
        {
            WriteValuesToConfigFile(File, ThisClass::getSectionName());
        };

        static ThisClass createObjectFromConfigFile(const ConfigFile &file, const std::string &section)
        {

            std::string Sec{ section + "." };

            auto coef = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(diffusionCoefficient));
            auto a = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(m_a));
            auto b = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(m_b));
            auto c = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(m_c));

            return ThisClass{ coef,a,b,c };
        }

        void GetValuesFromConfigFile(const ConfigFile &file, const std::string &section)
        {
            diffusionCoefficient = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(diffusionCoefficient));
            m_a = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(m_a));
            m_b = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(m_b));
            m_c = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(m_c));

        };
        void GetValuesFromConfigFile(const ConfigFile &file)
        {
            GetValuesFromConfigFile(file, ThisClass::getSectionName());
        };
        mxArray& createMATLABarray() const
        {

            mxArray &mxStruct = MATLABFileHandler::createMATLABStruct();
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(diffusionCoefficient), diffusionCoefficient);
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(m_a), m_a);
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(m_b), m_b);
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(m_c), m_c);
            return mxStruct;
        }
        void WriteValuesToMATLABFile(MATLABFileHandler &file, const std::string &section) const
        {
            mxArray& data = createMATLABarray();
            file.insertArrayIntoMAT(data, section);
        };
        void WriteValuesToMATLABFile(mxArray &mxStruct, const std::string &section) const
        {
            mxArray& data = createMATLABarray();
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, section, data);
        };

        // Access the A
        inline const prec& GetA(void) const noexcept { return(m_a); }
        inline void SetA(const prec& a) noexcept { m_a = a; }

        // Access the B
        inline const prec& GetB(void) const noexcept { return(m_b); }
        inline void SetB(const prec& b)	noexcept { m_b = b; }

        // Access the C
        inline const prec& GetC(void) const noexcept { return(m_c); }
        inline void SetC(const prec& c) noexcept { m_c = c; }

        // Access the DiffusionCoefficient
        inline const prec& GetDiffusionCoefficient(void) const	noexcept { return(diffusionCoefficient); }
        inline void SetDiffusionCoefficient(const prec& diffusionCoefficient) noexcept { diffusionCoefficient = diffusionCoefficient; }


    };
}