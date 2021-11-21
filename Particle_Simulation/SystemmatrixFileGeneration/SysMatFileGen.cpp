//We will not access floating point env.
//#pragma STDC FENV_ACCESS off
//#include <cfenv>
//#include <cerrno>

//Setup
#include "General/Setup.h"

//Command Options

#include "General/GeneralDefines.h"
#include "General/Setup.h"
#include "Simulator/SimulationManager.h"
//Starting Archive
#include <SerAr/ConfigFile/ConfigFile_Archive.h>

#include <variant>

#include "Settings/PhantomSettings.h"
#include "SystemmatrixFileGeneration/SysMatFileGen_Input.h"
#include "SysMatFileGen.h"

using AppTraits      = SimulationApplication::SimulationManagerTraits<SimulationApplication::SimulationManager<PREC>>;
using AppInputParams = SysMatFileGen_Input<AppTraits>;

int main(int argc, char** argv)
{
    using IField = Properties::IField;

    AppInputParams in_params(argc, argv);
    Settings::SimulationManagerSettings<double> SimManSet{in_params.getAppParams()};

    auto &var = (SimManSet.getFieldProperties().getFieldPropVariant()).getVariant();

    //auto& var = test.getVariant();

    typedef std::remove_reference<decltype(var)>::type FieldVar;

    std::visit(
        [&in_params](FieldVar&& arg) {
            if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Zero>::type>(arg)) 
            {
                SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Zero>::type>(in_params);
            }
        else if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Constant>::type>(arg))
            {
            SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Constant>::type>(in_params);
            }
        else if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Sinusoidal>::type>(arg))
            {
            SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Sinusoidal>::type>(in_params);
            }
        else if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Lissajous>::type>(arg)) 
            {
            SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Lissajous>::type>(in_params);      
            }
        else if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Triangular>::type>(arg))
            {
            SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Triangular>::type>(in_params);
            }
        else if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Rectangular>::type>(arg))
            {
            SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Rectangular>::type>(in_params);
            }
        else if (std::holds_alternative<SettingsFileGen::field_enum_property_mapping<IField::Field_Sinc>::type>(arg))
            {
            SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Sinc>::type>(in_params);
            }
        },var);
        return 0;
}
