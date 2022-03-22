#
# Program: STEPToPoints
#
# Description:
#
# The program STEPToPoints converts solids contained in STEP files into point clouds by regular sampling.
#
# Copyright(C) 2022 Alexander Leutgeb
#
# This library is free software; you can redistribute it and / or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110 - 1301  USA
#

cmake_minimum_required (VERSION 2.8)

macro(update_target_library LIBRARIES SEARCH REPLACE)
	foreach (LIBRARY ${LIBRARIES})
		get_target_property(TMP ${LIBRARY} INTERFACE_LINK_LIBRARIES)
		list(LENGTH TMP LEN)
		if(${LEN} GREATER 0)
			list(TRANSFORM TMP REPLACE "${SEARCH}" "${REPLACE}")
			set_target_properties(${LIBRARY} PROPERTIES INTERFACE_LINK_LIBRARIES "${TMP}")
		endif()
	endforeach()
endmacro()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project (STEPToPoints)

#
# begin of configuration section
# change values according to path structure in OpenCASCADE installation
#
set(OPENCASCADE_PREFIX "C:/OpenCASCADE-7.5.0-vc14-64" CACHE PATH "OpenCASCADE prefix")
if(WIN32)
	set(CMAKE_INSTALL_BINDIR "." CACHE PATH "User executables (bin)")
endif()
#
# end of configuration section
#

set(CMAKE_PREFIX_PATH "${OPENCASCADE_PREFIX};${CMAKE_PREFIX_PATH}")

find_package(OpenCASCADE)
find_package(OCE)

if(OpenCASCADE_WITH_TBB)
	set(TBB_LIBRARY_PREFIX "${OPENCASCADE_PREFIX}/tbb_2017.0.100/lib/intel64/vc14" CACHE PATH "TBB library directory")
	set(TBB_BIN_PREFIX "${OPENCASCADE_PREFIX}/tbb_2017.0.100/bin/intel64/vc14" CACHE PATH "TBB binary directory")
endif()
if(OpenCASCADE_WITH_FREEIMAGE)
	set(FREEIMAGE_LIBRARY_PREFIX "${OPENCASCADE_PREFIX}/freeimage-3.17.0-vc14-64/lib" CACHE PATH "FreeImage library directory")
	set(FREEIMAGE_BIN_PREFIX "${OPENCASCADE_PREFIX}/freeimage-3.17.0-vc14-64/bin" CACHE PATH "FreeImage binary directory")
	option(FREEIMAGE_SHARED "link FreeImage as shared library" ON)
endif()
if(OpenCASCADE_WITH_FFMPEG)
	set(FFMPEG_LIBRARY_PREFIX "${OPENCASCADE_PREFIX}/ffmpeg-3.3.4-64/lib" CACHE PATH "FFmpeg library directory")
	set(FFMPEG_BIN_PREFIX "${OPENCASCADE_PREFIX}/ffmpeg-3.3.4-64/bin" CACHE PATH "FFmpeg binary directory")
	option(FFMPEG_SHARED "link FFmpeg as shared libraries" ON)
endif()
if(OpenCASCADE_WITH_FREETYPE)
	set(FREETYPE_LIBRARY_PREFIX "${OPENCASCADE_PREFIX}/freetype-2.5.5-vc14-64/lib" CACHE PATH "FreeType library directory")
	set(FREETYPE_BIN_PREFIX "${OPENCASCADE_PREFIX}/freetype-2.5.5-vc14-64/bin" CACHE PATH "FreeType binary directory")
	option(FREETYPE_SHARED "link FreeType as shared library" ON)
endif()

include_directories(${OpenCASCADE_INCLUDE_DIR} ${OCE_INCLUDE_DIRS})

# OpenCASCADE-config.cmake doesn't define this option yet, so define it here instead
option(OpenCASCADE_WITH_OPENVR "whether OpenCASCADE was built with OpenVR support" ON)

if(OpenCASCADE_WITH_OPENVR)
	set(OPENVR_LIBRARY_PREFIX "${OPENCASCADE_PREFIX}/openvr-1.14.15-64/lib/win64" CACHE PATH "OpenVR library directory")
	set(OPENVR_BIN_PREFIX "${OPENCASCADE_PREFIX}/openvr-1.14.15-64/bin/win64" CACHE PATH "OpenVR binary directory")
endif()

if(NOT OpenCASCADE_BUILD_SHARED_LIBS)
	if(OpenCASCADE_WITH_TBB)
		link_directories(${TBB_LIBRARY_PREFIX})
	endif()
	if(OpenCASCADE_WITH_FREEIMAGE)
		link_directories(${FREEIMAGE_LIBRARY_PREFIX})
	endif()
	if(OpenCASCADE_WITH_FFMPEG)
		link_directories(${FFMPEG_LIBRARY_PREFIX})
	endif()
	if(OpenCASCADE_WITH_FREETYPE)
		link_directories(${FREETYPE_LIBRARY_PREFIX})
	endif()
endif()

set(LIBS TKernel TKMath TKBRep TKLCAF TKXDESTEP TKXCAF TKMesh TKTopAlgo TKSTL TKG3d TKG2d TKGeomBase TKCDF TKXSBase TKSTEP TKSTEPBase TKSTEPAttr TKShHealing TKVCAF TKCAF TKService TKV3d TKGeomAlgo TKSTEP209 TKBO TKHLR)

if(WIN32)
	#
	# update libraries for imported OpenCASCADE targets (modules),
	# because they have predefined paths from OpenCASCADE build
	#
	if(OpenCASCADE_WITH_TBB)
		update_target_library("${LIBS}" "^.+/tbbmalloc\\\\.lib$" "${TBB_LIBRARY_PREFIX}/tbbmalloc.lib")
		update_target_library("${LIBS}" "^.+/tbb\\\\.lib$" "${TBB_LIBRARY_PREFIX}/tbb.lib")
	endif()
	if(OpenCASCADE_WITH_FREEIMAGE)
		update_target_library("${LIBS}" "^.+/FreeImage\\\\.lib$" "${FREEIMAGE_LIBRARY_PREFIX}/FreeImage.lib")
	endif()
	if(OpenCASCADE_WITH_FFMPEG)
		update_target_library("${LIBS}" "^.+/avcodec\\\\.lib$" "${FFMPEG_LIBRARY_PREFIX}/avcodec.lib")
		update_target_library("${LIBS}" "^.+/avformat\\\\.lib$" "${FFMPEG_LIBRARY_PREFIX}/avformat.lib")
		update_target_library("${LIBS}" "^.+/swscale\\\\.lib$" "${FFMPEG_LIBRARY_PREFIX}/swscale.lib")
		update_target_library("${LIBS}" "^.+/avutil\\\\.lib$" "${FFMPEG_LIBRARY_PREFIX}/avutil.lib")
	endif()
	if(OpenCASCADE_WITH_FREETYPE)
		update_target_library("${LIBS}" "^.+/freetype\\\\.lib$" "${FREETYPE_LIBRARY_PREFIX}/freetype.lib")
	endif()
	if(OpenCASCADE_WITH_OPENVR)
		update_target_library("${LIBS}" "^.+/openvr_api\\\\.lib$" "${OPENVR_LIBRARY_PREFIX}/openvr_api.lib")
	endif()
endif()

add_executable(STEPToPoints STEPToPoints.cpp)

target_link_libraries(STEPToPoints ${LIBS})

include(GNUInstallDirs)

install(TARGETS STEPToPoints)

if(WIN32)
	option(INSTALL_DEPENDENCIES "whether to install dependent libraries or not" ON)
else()
	option(INSTALL_DEPENDENCIES "whether to install dependent libraries or not" OFF)
endif()

if(INSTALL_DEPENDENCIES)
	if(WIN32)
		if(OpenCASCADE_BUILD_SHARED_LIBS)
			foreach(LIB ${LIBS} TKPrim)
				# installing imported targets as TARGETS is not possible (see https://gitlab.kitware.com/cmake/cmake/-/issues/14311)
				# so we must use FILES mode
				install(FILES $<TARGET_FILE:${LIB}> TYPE BIN)
			endforeach()
		endif()
		if(OpenCASCADE_WITH_TBB)
			install(FILES 
				"${TBB_BIN_PREFIX}/tbb.dll"
				"${TBB_BIN_PREFIX}/tbbmalloc.dll"
				TYPE BIN)
		endif()
		if(OpenCASCADE_WITH_FREEIMAGE AND FREEIMAGE_SHARED)
			install(FILES 
				"${FREEIMAGE_BIN_PREFIX}/FreeImage.dll"
				TYPE BIN)
		endif()
		if(OpenCASCADE_WITH_FREETYPE AND FREETYPE_SHARED)
			install(FILES 
				"${FREETYPE_BIN_PREFIX}/freetype.dll"
				TYPE BIN)
		endif()
		if(OpenCASCADE_WITH_FFMPEG AND FFMPEG_SHARED)
			install(FILES 
				"${FFMPEG_BIN_PREFIX}/avcodec-57.dll"
				"${FFMPEG_BIN_PREFIX}/avformat-57.dll"
				"${FFMPEG_BIN_PREFIX}/swscale-4.dll"
				"${FFMPEG_BIN_PREFIX}/avutil-55.dll"
				TYPE BIN)
		endif()
		if(OpenCASCADE_WITH_OPENVR)
			install(FILES 
				"${OPENVR_BIN_PREFIX}/openvr_api.dll"
				TYPE BIN)
		endif()
	endif()
endif()
