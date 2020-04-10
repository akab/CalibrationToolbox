############################################################################
# <summary> Macros and Functions for Cmake PDFProofing software. </summary>
# <date>    2019-02-27          </date>
# <author>  Valerio Sanelli </author>
# <email>   valerio.sanellio@seeone.org</email>
############################################################################

##----- Draw the library logo
macro( print_title )
message("TopScan")
endmacro()

##----- Add cmake modules from cmake/modules
#       git@github.com:rpavlik/cmake-modules.git
macro( include_modules )
	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})
	set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH})
endmacro()

##----- Make a version file containing the current version from git.
function( get_version vers_major vers_minor vers_patch)
	#
	include(GetGitRevisionDescription)
	git_describe(VERSION --tags )

	#parse the version information into pieces.
	string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" VERSION_MAJOR "${VERSION}")
	string(REGEX REPLACE "^[0-9]+\\.([0-9]+).*" "\\1" VERSION_MINOR "${VERSION}")
	string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" VERSION_PATCH "${VERSION}")
	string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1" VERSION_SHA1 "${VERSION}")
	set(VERSION_SHORT "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

	# set variables for external cmake
	set(vers_major ${VERSION_MAJOR} PARENT_SCOPE)
	set(vers_minor ${VERSION_MINOR} PARENT_SCOPE)
	set(vers_patch ${VERSION_PATCH} PARENT_SCOPE)
endfunction()

##----- Create a Visual Studio Filter from _source_list
macro(create_VS_filter _source_list _filter_name)
	foreach(_source IN ITEMS ${_source_list})
    	source_group(${_filter_name} FILES "${_source}")
	endforeach()
endmacro()

##----- Get list of all subdirectories in a folder
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()