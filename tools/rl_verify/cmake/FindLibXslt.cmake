#
# Stub FindLibXslt.cmake — workaround for missing libxslt in this MinGW setup.
#
# RL 0.7.0 requires LibXslt in CMake because rl::xml's link interface mentions
# LibXslt::LibXslt, but the ONLY consumer is rl::xml::Stylesheet (XSLT transforms).
# rl::math / rl::mdl / rl::kin never reference libxslt symbols — XmlFactory uses
# rl::xml::Document/DomParser only, which are pure libxml2.
#
# msys2 ucrt64 (the local libxml2 provider) does not ship libxslt, and the
# msys2 mirrors were unreachable at build time. So we provide a fake
# libxslt (see fakexslt/ in this directory) that keeps CMake's REQUIRED
# checks happy and compiles rl::xml::Stylesheet, without adding link flags.
#
# See docs/rl-build-notes.md for the full story.
#

set(LIBXSLT_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/fakexslt" CACHE PATH
    "fake libxslt include dir (no real libxslt installed)")
set(LIBXSLT_LIBRARY "" CACHE FILEPATH
    "libxslt stub library (no real libxslt installed)")
set(LIBXSLT_LIBRARIES "" CACHE STRING "libxslt stub libraries")
set(LIBXSLT_INCLUDE_DIRS "${LIBXSLT_INCLUDE_DIR}")
set(LIBXSLT_VERSION "0.0.0-stub")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    LibXslt
    REQUIRED_VARS LIBXSLT_INCLUDE_DIR
    VERSION_VAR LIBXSLT_VERSION
)

if(LibXslt_FOUND AND NOT TARGET LibXslt::LibXslt)
    add_library(LibXslt::LibXslt INTERFACE IMPORTED)
    set_target_properties(LibXslt::LibXslt PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${LIBXSLT_INCLUDE_DIR}")
endif()

mark_as_advanced(LIBXSLT_INCLUDE_DIR LIBXSLT_LIBRARY)
