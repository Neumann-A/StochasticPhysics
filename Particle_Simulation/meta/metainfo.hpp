//#include <SerAr/Core/MyCEL_MetaInfoSerializer.hpp>
#pragma once
#include <MyCEL/types/metainfo.hpp>
#include <SerAr/Core/MyCEL_MetaInfoSerializer.hpp>
#include <SerAr/Core/ArchiveHelper.h>

extern const MyCEL::git_metainfo& getGitMetaInfo(); 
extern const MyCEL::build_metainfo& getBuildMetaInfo(); 

struct ProjectMetaInfo {};

template<typename Archive>
void serialize(ProjectMetaInfo&, Archive& ar) {
    if constexpr(SerAr::IsOutputArchive<std::remove_cvref_t<Archive>>)
    {
        ar(Archives::createNamedValue("build",getBuildMetaInfo()));
        ar(Archives::createNamedValue("git",getGitMetaInfo()));
    }
}