
#Warnings from https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md
set(CLANG_GCC_WARNINGS
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
    -Weffc++
)

set(GCC_SILENCED 
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
list(APPEND CLANG_GCC_WARNINGS
      -Wmisleading-indentation # warn if identation implies blocks where blocks do not exist
      -Wduplicated-cond # warn if if / else chain has duplicated conditions
      -Wduplicated-branches # warn if if / else branches have duplicated code
      -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
      -Wuseless-cast # warn if you perform a cast to the same type
)

add_compile_options(${CLANG_GCC_WARNINGS})
add_compile_options(${GCC_SILENCED})