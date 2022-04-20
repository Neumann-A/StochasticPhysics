set(CPP_FILES_Fields Fields/FieldList.cpp)
#set(INL_FILES_Fields Fields/SinusoidalField.inl)
set(H_FILES_Fields  Fields/FieldList.h
                    Fields/ConstantField.h
                    Fields/ForceField.h
                    Fields/InteractionField.h
                    Fields/LissajousField.h
                    Fields/SinusoidalField.h
                    Fields/TriangularField.h
                    Fields/ZeroField.h
                    Fields/RectangularField.h
                    Fields/SincField.h
                    Fields/ModulatedSincField.h
                    )
source_group("Fields"        FILES ${CPP_FILES_Fields} ${INL_FILES_Fields} ${H_FILES_Fields})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Fields})
set(INL_FILES ${INL_FILES} ${INL_FILES_Fields})
set(H_FILES ${H_FILES} ${H_FILES_Fields})