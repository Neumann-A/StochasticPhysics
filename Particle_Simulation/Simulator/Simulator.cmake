set(CPP_FILES_Simulator Simulator/SimulationApplication.cpp
                        Simulator/Simulator.cpp)
set(H_FILES_Simulator   Simulator/ISingleParticleSimulator.h
                        Simulator/SimulationManager.h
                        Simulator/SimulationManagerTraits.h
                        Simulator/SingleParticleSimulator.h
                        Simulator/Simulator.hpp
                        )

configure_file("Simulator/Simulator_arch.hpp.in" "Simulator/Simulator_arch.hpp")

foreach(ARCH IN LISTS StoPhys_ARCH_LIST)
    set(current_arch_target "SPhys_Simulator_${ARCH}")
    list(APPEND Simulator_ARCH_TARGETS "${current_arch_target}")
    configure_file("Simulator/Simulator_arch.cpp.in" "Simulator/Simulator_${ARCH}.cpp")
    add_library(${current_arch_target} SHARED "Simulator/Simulator_${ARCH}.cpp" $<TARGET_OBJECTS:SimulationCommon>)
    target_include_directories(${current_arch_target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
    target_compile_definitions(${current_arch_target} PRIVATE SimulationCommon_EXPORTS)
    target_link_libraries(${current_arch_target} PRIVATE MyCEL::MyCEL 
                                                         SimulationArchive 
                                                         InstructionsSets 
                                                         Threads::Threads 
                                                         ${PROJECT_NAME}_CommonTarget)
    if(ARCH_${ARCH}_FLAG)
        target_compile_options(${current_arch_target} PRIVATE "${ARCH_${ARCH}_FLAG}")
    endif()

    install(TARGETS ${current_arch_target}
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        )

    set(_pref_dep ${current_arch_target})
    list(APPEND ARCH_DEFS WITH_${ARCH})
    list(APPEND ARCH_DLL_TARGETS ${current_arch_target})

    if(StoPhys_LIMITED_RESOURCE)
        set_property(TARGET ${current_arch_target} PROPERTY JOB_POOL_LINK one_job)
        set_property(TARGET ${current_arch_target} PROPERTY JOB_POOL_COMPILE one_job)
    endif()
endforeach()

source_group("Simulator"        FILES ${CPP_FILES_Simulator} ${INL_FILES_Simulator} ${H_FILES_Simulator})
set(CPP_FILES ${CPP_FILES} )
set(INL_FILES ${INL_FILES} ${INL_FILES_Simulator})
set(H_FILES ${H_FILES} ${H_FILES_Simulator})