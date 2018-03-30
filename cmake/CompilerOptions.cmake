enable_language(CXX)
 
 message(STATUS "Reported configuration types:  ${CMAKE_CONFIGURATION_TYPES}")
 message(STATUS "Reported compiler:  ${CMAKE_CXX_COMPILER_ID}")

 if (MSVC AND NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  # using Visual Studio C++
  set(MSVC_WARNINGS "/W4;/w44640;/w14242;/w14254;/w14263;/w14265;/w14287;/we4289;/w14296;/w14311;/w14545;/w14546;/w14547;/w14549;/w14555;/w14619;/w14640;/w14826;/w14905;/w14906;/w14928")
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  ADD_DEFINITIONS(/bigobj)
  ADD_DEFINITIONS(/Zi)
  set(MY_DEBUG_OPTIONS "/W4;/arch:AVX2;/bigobj;/EHsc;/std:c++latest")
  
  if ( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # using Clang-Cl
	set(MSVC_CLANG_WARNINGS "-Wall;-Wextra;-Wshadow;-Wnon-virtual-dtor;-Wold-style-cast;-Wcast-align;-Wunused;-Woverloaded-virtual;-Wpedantic;-Wconversion;-Wsign-conversion;-Wduplicate-enum;-Wnull-dereference;-Wdouble-promotion;-Wformat=2;-Weffc++" )
	set(MY_RELEASE_OPTIONS "${MSVC_WARNINGS};/Ox;/Oi;/Ot;/Oy;/fp:fast;/arch:AVX2;/bigobj;-fmacro-backtrace-limit=0;-fcolor-diagnostics;/std:c++latest;/EHsc")
	#set(CMAKE_CXX_FLAGS "/DWIN32 /D_WINDOWS /W4 /GR /EHsc" CACHE INTERNAL "CMAKE_CXX_FLAGS" FORCE )
	#set(CMAKE_CXX_FLAGS_DEBUG "/MDd /Zi /Ob0 /Od /RTC1" CACHE INTERNAL "CMAKE_CXX_FLAGS_DEBUG" FORCE )
	#set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /DNDEBUG" CACHE INTERNAL "CMAKE_CXX_FLAGS_RELEASE" FORCE )
	#set(CMAKE_CXX_FLAGS_MINSIZEREL "/MD /O1 /Ob1 /DNDEBUG" CACHE INTERNAL "CMAKE_CXX_FLAGS_MINSIZEREL" FORCE )
	#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD /Zi /O2 /Ob1 /DNDEBUG" CACHE INTERNAL "CMAKE_CXX_FLAGS_RELWITHDEBINFO" FORCE )
	add_compile_options("$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
	add_compile_options("$<$<CONFIG:RELEASE>:${MY_RELEASE_OPTIONS}>")
	add_compile_options("$<$<CONFIG:RELWITHDEBINFO>:${MY_RELEASE_OPTIONS}>")
	add_compile_options(-Xclang "-fno-aligned-allocation;${MSVC_CLANG_WARNINGS}")
	message(STATUS "Clang-Cl compiler switches set!")
  else()
	set(MY_DEBUG_OPTIONS "${MSVC_WARNINGS};/Ox;/Oi;/Ot;/Oy;/GL;/MP;/fp:fast;/arch:AVX2;/bigobj;/std:c++latest")
	set(MY_RELEASE_OPTIONS "${MSVC_WARNINGS};/Ox;/Oi;/Ot;/Oy;/fp:fast;/arch:AVX2;/bigobj;/std:c++latest;/EHsc")
	add_compile_options("$<$<CONFIG:DEBUG>:${MY_DEBUG_OPTIONS}>")
	add_compile_options("$<$<CONFIG:RELEASE>:${MY_RELEASE_OPTIONS}>")
	add_compile_options("$<$<CONFIG:RELWITHDEBINFO>:${MY_RELEASE_OPTIONS}>")
	message(STATUS "VS C++ compiler switches set!")
  endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" AND NOT MSVC)
    set(CLANG_WARNINGS "-Wall;-Wextra;-Wshadow;-Wnon-virtual-dtor;-Wold-style-cast;-Wcast-align;-Wunused;-Woverloaded-virtual;-Wpedantic;-Wconversion;-Wsign-conversion;-Wmisleading-indentation;-Wduplicated-cond;-Wduplicated-branches;-Wlogical-op;-Wnull-dereference;-Wuseless-cast;-Wdouble-promotion;-Wformat=2;-Weffc++" )
	add_compile_options("-std=c++1z")
	if(!MINGW)
	add_compile_options("-stdlib=libc++")
	endif()
	add_compile_options("-O3")
	add_compile_options("-Ofast")
	add_compile_options("-fvectorize")
	#add_compile_options("-fsanitize=address -fsanitize=thread -fsanitize=leak -fsanitize=undefined ")
	add_compile_options("-march=haswell")
	#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lc++ -lc++experimental")
	add_compile_options("${CLANG_WARNINGS}")
	add_compile_options("-m64")
	#add_compile_options("-fsanitize=undefined")
	#add_definitions("-v")
	#add_compile_options("-pedantic")
	#add_compile_options("-flto") # does not work
	#add_compile_options("-nostdinc")
	#add_compile_options("-isystem/usr/include/c++/4.9")
	#add_compile_options("-isystem/usr/include/clang/4.0.0/include")
	#add_compile_options("-i/usr/local/include/c++/v1")
	#add_compile_options("-isystem/usr/local/include")
	#add_compile_options("-isystem/usr/include/x86_64-linux-gnu/")
	#add_compile_options("-isystem/usr/include/x86_64-linux-gnu/c++/4.9")
	#add_compile_options("-iisystem/usr/include/eigen3")
	add_compile_options("-v")
	#add_compile_options("-lc++abi")
  # using Clang
  message(STATUS "Clang compiler switches set!")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
 
  # using GCC
    add_compile_options("-std=c++14")
  if(!MINGW)
	add_compile_options("-stdlib=libc++")
	#add_compile_options("-lc++abi")
    message(STATUS "GCC compiler switches set!")
  endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  # using Intel C++
  message(STATUS "Intel compiler switches not set!")

else("${CMAKE_CXX_COMPILER_ID}" STREQUAL "")
  message(STATUS "CMAKE_CXX_COMPILER_ID not set!")
endif()