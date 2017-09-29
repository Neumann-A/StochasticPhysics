OPTION(General_Benchmarks "Enable building of benchmarks" ON)
OPTION(General_Tests "Enable building of tests" ON)

if (CMAKE_COMPILER_IS_GNUCC)
  option(General_Coverage "Enable coverage reporting for gcc/clang" FALSE)
  
    if (General_Coverage)
    add_compile_options(--coverage -O0)
  endif()
endif()
