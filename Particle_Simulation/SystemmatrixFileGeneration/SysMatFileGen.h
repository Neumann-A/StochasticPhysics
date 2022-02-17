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

using AppTraits      = SimulationApplication::SimulationManagerTraits<SimulationApplication::SimulationManager<PREC>>;
using AppInputParams = SysMatFileGen_Input<AppTraits>;

namespace SettingsFileGen
{

    using IField = Properties::IField;
    template <IField value>
    struct field_enum_property_mapping
    {
        using type = typename Selectors::FieldSelector<value>::template FieldParameters<PREC>;
    };

#define SYSMATFILEGENMAKRO(parameterType)                                                                              \
    template <>                                                                                                        \
    void SimFileGen<parameterType>(AppInputParams & in_params, Settings::SimulationManagerSettings<PREC> SimManSet)    \
    {                                                                                                                  \
                                                                                                                       \
        int i                                     = 0;                                                                 \
        auto& particleProvider                    = SimManSet.getProvider();                                           \
        particleProvider.saveParticlesInSameFile = false;                                                             \
        parameterType& val = (SimManSet.getFieldProperties()).getFieldParameters<parameterType::TypeOfField>();        \
        const auto offsetfield{val.OffsetField};                                                                       \
                                                                                                                       \
        if (!SysMatFileGen_Input<ThisAppTraits>::options.matrix_file.empty()) {                                        \
            Settings::SystemMatrixSettings<PREC> SysMatSet{in_params.getSysMatParams()};                               \
            auto VoxelFieldTupleVector = SysMatSet.generateFieldsAndVoxels();                                          \
                                                                                                                       \
            auto SlicesSys                = SysMatSet.getSlices();                                                     \
            std::string SysFileSaveFolder = "SystemMatrixFiles_" + std::to_string(SlicesSys(0)) + "x" +                \
                std::to_string(SlicesSys(1)) + "x" + std::to_string(SlicesSys(0));                                     \
            fs::create_directories(SysMatFileGen_Input<ThisAppTraits>::options.save_directory.string() + "/" +         \
                                   SysFileSaveFolder);                                                                 \
                                                                                                                       \
            const auto& filepath_mat = SysMatFileGen_Input<ThisAppTraits>::options.matrix_file;                        \
                                                                                                                       \
            for (auto& VoxelFieldTuple : VoxelFieldTupleVector) {                                                      \
                const auto& Field = std::get<1>(VoxelFieldTuple);                                                      \
                val.OffsetField   = Field + offsetfield;                                                               \
                i++;                                                                                                   \
                const std::filesystem::path filename{                                                                  \
                    SysMatFileGen_Input<ThisAppTraits>::options.save_directory.string() + "\\" + SysFileSaveFolder +   \
                    "\\" + SysMatFileGen_Input<ThisAppTraits>::options.output_file_name.filename().string() +          \
                    std::to_string(i) + filepath_mat.extension().string()};                                            \
                SerAr::AllFileOutputArchiveWrapper archive(filename, SerAr::ArchiveOutputMode::Overwrite);             \
                archive(SimManSet);                                                                                    \
            }                                                                                                          \
        }                                                                                                              \
        if (!SysMatFileGen_Input<ThisAppTraits>::options.phantom_file.empty()) {                                       \
                                                                                                                       \
            Settings::PhantomSettings<PREC> Phantom{in_params.getPhantom()};                                           \
            auto VoxelFieldTupleVector = Phantom.generateFieldsAndVoxels();                                            \
                                                                                                                       \
            auto Slices                   = Phantom.getSlices();                                                       \
            std::string PhantomSaveFolder = "Phantom_" + std::to_string(Slices(0)) + "x" + std::to_string(Slices(1)) + \
                "x" + std::to_string(Slices(0));                                                                       \
            fs::create_directories(PhantomSaveFolder);                                                                 \
                                                                                                                       \
            const auto& filepath_ph = SysMatFileGen_Input<ThisAppTraits>::options.phantom_file;                        \
                                                                                                                       \
            for (auto& VoxelFieldTuple : VoxelFieldTupleVector) {                                                      \
                const auto& Field = std::get<1>(VoxelFieldTuple);                                                      \
                val.OffsetField   = Field + offsetfield;                                                               \
                i++;                                                                                                   \
                const std::filesystem::path filename{                                                                  \
                    PhantomSaveFolder + "\\" +                                                                         \
                    SysMatFileGen_Input<ThisAppTraits>::options.output_file_name.filename().string() +                 \
                    std::to_string(i) + filepath_ph.extension().string()};                                             \
                SerAr::AllFileOutputArchiveWrapper archive(filename, SerAr::ArchiveOutputMode::Overwrite);             \
                archive(SimManSet);                                                                                    \
            }                                                                                                          \
        }                                                                                                              \
    }

    template <typename T>
    void SimFileGen(AppInputParams& in_params, Settings::SimulationManagerSettings<PREC> SimManSet)
    {
    }

    SYSMATFILEGENMAKRO(field_enum_property_mapping<IField::Field_Constant>::type)
    SYSMATFILEGENMAKRO(field_enum_property_mapping<IField::Field_Sinusoidal>::type)
    SYSMATFILEGENMAKRO(field_enum_property_mapping<IField::Field_Lissajous>::type)
    SYSMATFILEGENMAKRO(field_enum_property_mapping<IField::Field_Triangular>::type)
    SYSMATFILEGENMAKRO(field_enum_property_mapping<IField::Field_Rectangular>::type)
    SYSMATFILEGENMAKRO(field_enum_property_mapping<IField::Field_Sinc>::type)
} // namespace SettingsFileGen
#undef SYSMATFILEGENMAKRO