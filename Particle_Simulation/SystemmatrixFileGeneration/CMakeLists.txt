add_executable(SysMatFileGen "SysMatFileGen.cpp"
                             "SysMatFileGen.h"
                             "SysMatFileGen_Input.cpp"
                             "SysMatFileGen_Input.h"
                             $<TARGET_OBJECTS:SimulationCommon>)

target_link_libraries(SysMatFileGen  
                        PRIVATE 
                                MyCEL::MyCEL 
                                SimulationArchive
                                Boost::program_options
                                Eigen3::Eigen
                                ${PROJECT_NAME}_CommonTarget
                     )