# - Try to find SuiteSparse
# Once done, this will define
#
#  SuiteSparse_FOUND - system has SuiteSparse
#  SuiteSparse_INCLUDE_DIRS - the SuiteSparse include directories
#  SuiteSparse_LIBRARIES - link these to use SuiteSparse

if (SuiteSparse_INCLUDE_DIRS AND SuiteSparse_LIBRARIES)
	# in cache already
	set(SuiteSparse_FOUND TRUE)
else (SuiteSparse_INCLUDE_DIRS AND SuiteSparse_LIBRARIES)
	include(LibFindMacros)

	# Dependencies
	# NOTE: SuiteSparse has no explicit dependencies.
	# libfind_package(SuiteSparse Dependencies)

	# Use pkg-config to get hints about paths
	libfind_pkg_check_modules(SuiteSparse_PKGCONF newmat)
	
	# Include dir
	find_path(SuiteSparse_INCLUDE_DIR
		NAMES suitesparse/cs.h
		PATHS 
			${SuiteSparse_PKGCONF_INCLUDE_DIRS}
			/usr/include
			/usr/local/include
	)

	# Finally the library itself
	find_library(SuiteSparse_LIBRARY
		NAMES cxsparse
		PATHS ${SuiteSparse_PKGCONF_LIBRARY_DIRS}
	)

	# Set the include dir variables and the libraries and let libfind_process do the rest.
	# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
	set(SuiteSparse_PROCESS_INCLUDES SuiteSparse_INCLUDE_DIR)
	set(SuiteSparse_PROCESS_LIBS SuiteSparse_LIBRARY)
	libfind_process(SuiteSparse)
	
	if (SuiteSparse_INCLUDE_DIRS AND SuiteSparse_LIBRARIES)
		set(SuiteSparse_FOUND TRUE)
	endif (SuiteSparse_INCLUDE_DIRS AND SuiteSparse_LIBRARIES)
endif (SuiteSparse_INCLUDE_DIRS AND SuiteSparse_LIBRARIES)
