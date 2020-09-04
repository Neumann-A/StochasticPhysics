message(STATUS "Loading clang++ using ninja toolchain!")
#Warnings from https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md
set(CLANG_WARNINGS
    -Wall
    -Wextra # reasonable and standard
    -Wshadow # warn the user if a variable declaration shadows one from a parent context
    -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps catch hard to track down memory errors
    -Wold-style-cast # warn for c-style casts
    -Wcast-align # warn for potential performance problem casts
    -Wunused # warn on anything being unused
    -Woverloaded-virtual # warn if you overload (not override) a virtual function
    -Wpedantic # warn if non-standard C++ is used
    -Wconversion # warn on type conversions that may lose data
    -Wsign-conversion # warn on sign conversions
    -Wnull-dereference # warn if a null dereference is detected
    -Wdouble-promotion # warn if float is implicit promoted to double
    -Wformat=2 # warn on security issues around functions that format output (ie printf)
    -Wmisleading-indentation
    #-Wduplicated-cond
    #-Wduplicated-branches
    #-Wlogical-op
    #-Wuseless-cast
    -Weffc++
)

set(CLANG_SILENCED 
    -Wno-c++98-compat
    -Wno-c++98-compat-pedantic
    -Wno-documentation-deprecated-sync
    -Wno-undef
    -Wno-documentation
    -Wno-global-constructors
    -Wno-exit-time-destructors
    -Wno-documentation-unknown-command
    -Wno-zero-as-null-pointer-constant
    -Wno-old-style-cast
    -Wno-switch-enum
    -Wno-cast-align
    -Wno-covered-switch-default
    -Wno-deprecated-dynamic-exception-spec
    -Wno-date-time
    -Wno-language-extension-token
    -Wno-extra-semi
    -Wno-reserved-id-macro
    -Wno-missing-noreturn
    -Wno-unused-template
)

add_compile_options(${CLANG_WARNINGS})
add_compile_options(${CLANG_SILENCED})
add_compile_options(-fcolor-diagnostics)

#add_compile_options("-stdlib=libc++")
add_compile_options("-O2")
add_compile_options("-Ofast")
add_compile_options("-fvectorize")
#add_compile_options("-march=skylake-avx512")
#string(APPEND CMAKE_EXE_LINKER_FLAGS "-static")
#string(APPEND CMAKE_EXE_LINKER_FLAGS "-static -lc++ -lc++abi -lc++experimental -lc++fs")
add_compile_options("${CLANG_WARNINGS}")
add_compile_options("-m64")
#add_compile_options("-fsanitize=undefined")
#add_definitions("-v")
#add_compile_options("-pedantic")
#add_compile_options("-flto=full") # does not work
add_compile_options("-funroll-loops")
add_compile_options("-ffp-contract=fast")
add_compile_options("-ffast-math")
add_compile_options("-g")
#add_compile_options("-nostdinc")
#add_compile_options("-isystem/usr/include/c++/4.9")
#add_compile_options("-isystem/usr/include/clang/4.0.0/include")
#add_compile_options("-i/usr/local/include/c++/v1")
#add_compile_options("-isystem/usr/local/include")
#add_compile_options("-isystem/usr/include/x86_64-linux-gnu/")
#add_compile_options("-isystem/usr/include/x86_64-linux-gnu/c++/4.9")
#add_compile_options("-iisystem/usr/include/eigen3")
#add_compile_options("-v")
#add_compile_options("-lc++abi")
if(DEFINED _VCPKG_INSTALLED_DIR AND EXISTS "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include")
    add_compile_options("--system-header-prefix=${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include")
endif()