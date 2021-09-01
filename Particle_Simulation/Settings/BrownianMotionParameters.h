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

        prec										_DiffusionCoefficient;
        
        prec										_a;
        prec										_b;
        prec										_c;


    public:

        explicit BrownianMotionParameters(const prec& difcoef, const prec& a, const prec& b, const prec& c) :
            _DiffusionCoefficient(difcoef), _a(a), _b(b), _c(c)
        {

        }

        static std::string getSectionName() { return std::string{ "BrownianMotionParameters" }; };

        void WriteValuesToConfigFile(ConfigFile &file, const std::string &section) const
        {
            file.addKeyValue(section, SVARNAME(_DiffusionCoefficient), BasicTools::toStringScientific(_DiffusionCoefficient));
            file.addKeyValue(section, SVARNAME(_a), BasicTools::toStringScientific(_a));
            file.addKeyValue(section, SVARNAME(_b), BasicTools::toStringScientific(_b));
            file.addKeyValue(section, SVARNAME(_c), BasicTools::toStringScientific(_c));
            
        };
        void WriteValuesToConfigFile(ConfigFile &File) const
        {
            WriteValuesToConfigFile(File, ThisClass::getSectionName());
        };

        static ThisClass createObjectFromConfigFile(const ConfigFile &file, const std::string &section)
        {

            std::string Sec{ section + "." };

            auto coef = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_DiffusionCoefficient));
            auto a = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_a));
            auto b = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_b));
            auto c = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_c));

            return ThisClass{ coef,a,b,c };
        }

        void GetValuesFromConfigFile(const ConfigFile &file, const std::string &section)
        {
            _DiffusionCoefficient = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_DiffusionCoefficient));
            _a = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_a));
            _b = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_b));
            _c = file.getNumericValueOfKeyInSection<prec>(section, SVARNAME(_c));

        };
        void GetValuesFromConfigFile(const ConfigFile &file)
        {
            GetValuesFromConfigFile(file, ThisClass::getSectionName());
        };
        mxArray& createMATLABarray() const
        {

            mxArray &mxStruct = MATLABFileHandler::createMATLABStruct();
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(_DiffusionCoefficient), _DiffusionCoefficient);
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(_a), _a);
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(_b), _b);
            MATLABFileHandler::insertSingleElementIntoStruct(mxStruct, SVARNAME(_c), _c);
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
        inline const prec& GetA(void) const noexcept { return(_a); }
        inline void SetA(const prec& a) noexcept { _a = a; }

        // Access the B
        inline const prec& GetB(void) const noexcept { return(_b); }
        inline void SetB(const prec& b)	noexcept { _b = b; }

        // Access the C
        inline const prec& GetC(void) const noexcept { return(_c); }
        inline void SetC(const prec& c) noexcept { _c = c; }

        // Access the DiffusionCoefficient
        inline const prec& GetDiffusionCoefficient(void) const	noexcept { return(_DiffusionCoefficient); }
        inline void SetDiffusionCoefficient(const prec& diffusionCoefficient) noexcept { _DiffusionCoefficient = diffusionCoefficient; }


    };
}