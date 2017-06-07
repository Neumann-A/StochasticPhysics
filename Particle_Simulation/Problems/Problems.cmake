include_directories(Problems)
file(GLOB CPP_FILES_Problems Problems/*.cpp)
file(GLOB INL_FILES_Problems Problems/*.inl)
file(GLOB H_FILES_Problems Problems/*.h)
source_group("Problems"        FILES ${CPP_FILES_Problems} ${INL_FILES_Problems} ${H_FILES_Problems})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems})
set(H_FILES ${H_FILES} ${H_FILES_Problems})

#Anisotropy in Problem
include_directories(Problems/Anisotropy)
file(GLOB CPP_FILES_ProblemAnisotropy Problems/Anisotropy/*.cpp)
file(GLOB INL_FILES_ProblemAnisotropy Problems/Anisotropy/*.inl)
file(GLOB H_FILES_ProblemAnisotropy Problems/Anisotropy/*.h)
source_group("Problems\\Anisotropy"        FILES ${CPP_FILES_ProblemAnisotropy} ${INL_FILES_ProblemAnisotropy} ${H_FILES_ProblemAnisotropy})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_ProblemAnisotropy})
set(INL_FILES ${INL_FILES} ${INL_FILES_ProblemAnisotropy})
set(H_FILES ${H_FILES} ${H_FILES_ProblemAnisotropy})

#Boundary_Condition
include_directories(Problems/Boundary_Condition)
file(GLOB CPP_FILES_Problems_Boundary Problems/Boundary_Condition/*.cpp)
file(GLOB INL_FILES_Problems_Boundary Problems/Boundary_Condition/*.inl)
file(GLOB H_FILES_Problems_Boundary Problems/Boundary_Condition/*.h)
source_group("Problems\\Boundary_Condition"        FILES ${CPP_FILES_Problems_Boundary} ${INL_FILES_Problems_Boundary} ${H_FILES_Problems_Boundary})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems_Boundary})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems_Boundary})
set(H_FILES ${H_FILES} ${H_FILES_Problems_Boundary})

#Definitions
include_directories(Problems/Definitions)
file(GLOB CPP_FILES_Problems_Definitions Problems/Definitions/*.cpp)
file(GLOB INL_FILES_Problems_Definitions Problems/Definitions/*.inl)
file(GLOB H_FILES_Problems_Definitions Problems/Definitions/*.h)
source_group("Problems\\Definitions"        FILES ${CPP_FILES_Problems_Definitions} ${INL_FILES_Problems_Definitions} ${H_FILES_Problems_Definitions})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems_Definitions})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems_Definitions})
set(H_FILES ${H_FILES} ${H_FILES_Problems_Definitions})

#Helpers
include_directories(Problems/Helpers)
file(GLOB CPP_FILES_Problems_Helpers Problems/Helpers/*.cpp)
file(GLOB INL_FILES_Problems_Helpers Problems/Helpers/*.inl)
file(GLOB H_FILES_Problems_Helpers Problems/Helpers/*.h)
source_group("Problems\\Helpers"        FILES ${CPP_FILES_Problems_Helpers} ${INL_FILES_Problems_Helpers} ${H_FILES_Problems_Helpers})
set(CPP_FILES ${CPP_FILES} ${CPP_FILES_Problems_Helpers})
set(INL_FILES ${INL_FILES} ${INL_FILES_Problems_Helpers})
set(H_FILES ${H_FILES} ${H_FILES_Problems_Helpers})