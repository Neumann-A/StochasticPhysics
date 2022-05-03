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
#include "SysMatFileGen.h"
#include "SystemmatrixFileGeneration/SysMatFileGen_Input.h"

using AppTraits      = SimulationApplication::SimulationManagerTraits<SimulationApplication::SimulationManager<PREC>>;
using AppInputParams = SysMatFileGen_Input<AppTraits>;
template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

int main(int argc, char** argv)
{
    using IField = Properties::IField;

    AppInputParams in_params(argc, argv);
    Settings::SimulationManagerSettings<PREC> SimManSet{in_params.getAppParams()};

    auto &var = SimManSet.getFieldProperties().fieldproperties.variant;
std::visit(overload{
        [](SettingsFileGen::field_enum_property_mapping<IField::Field_Zero>::type){
                    std::puts("Zero field not supportet");},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Constant>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Constant>::type>(std::move(in_params),SimManSet);},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Sinusoidal>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Sinusoidal>::type>(std::move(in_params),SimManSet);},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Lissajous>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Lissajous>::type>(std::move(in_params),SimManSet);},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Triangular>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Triangular>::type>(std::move(in_params),SimManSet);},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Rectangular>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Rectangular>::type>(std::move(in_params),SimManSet);},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Sinc>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Sinc>::type>(std::move(in_params),SimManSet);},
        [&in_params,&SimManSet](SettingsFileGen::field_enum_property_mapping<IField::Field_Modsinc>::type&){
                    SettingsFileGen::SimFileGen<SettingsFileGen::field_enum_property_mapping<IField::Field_Modsinc>::type>(std::move(in_params),SimManSet);}
},var);
        return 0;
}
