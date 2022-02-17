// This file is generated. Dont modify it!
#include "meta/metainfo.hpp"

#define BUILD_DATE "@PROJECT_DATE@"
#define BUILD_TIME "@PROJECT_TIME@"
#define BUILD_GIT_SHA "@PROJECT_GIT_SHA@"
#define BUILD_GIT_DESCRIBE "@PROJECT_GIT_DESCRIBE@"
#define BUILD_GIT_BRANCH "@PROJECT_GIT_BRANCH@"

const MyCEL::git_metainfo& getGitMetaInfo() {
    static MyCEL::git_metainfo git_info{.branch=BUILD_GIT_BRANCH, .sha=BUILD_GIT_SHA, .describe=BUILD_GIT_DESCRIBE};
    return git_info;
}; 
const MyCEL::build_metainfo& getBuildMetaInfo() {
    static MyCEL::build_metainfo build_info{.date = BUILD_DATE, .time = BUILD_TIME};
    return build_info;
}
