find_path( THRUST_INCLUDE_DIR
        HINTS /usr/local/cuda-8.0/include /usr/local/include
        NAMES thrust/version.h
        DOC "Thrust headers"
        )
if( THRUST_INCLUDE_DIR )
    list( REMOVE_DUPLICATES THRUST_INCLUDE_DIR )
    include_directories( ${THRUST_INCLUDE_DIR} )
endif( THRUST_INCLUDE_DIR )



# Find thrust version
file( STRINGS ${THRUST_INCLUDE_DIR}/thrust/version.h
        version
        REGEX "#define THRUST_VERSION[ \t]+([0-9x]+)"
        )
string( REGEX REPLACE "#define THRUST_VERSION[ \t]+" "" version $
        {version} )

string( REGEX MATCH "^[0-9]" major ${version} )
string( REGEX REPLACE "^${major}00" "" version ${version} )
string( REGEX MATCH "^[0-9]" minor ${version} )
string( REGEX REPLACE "^${minor}0" "" version ${version} )
set( THRUST_VERSION "${major}.${minor}.${version}")

# Check for required components
set( THRUST_FOUND TRUE )

include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Thrust
        REQUIRED_VARS
        THRUST_INCLUDE_DIR
        VERSION_VAR
        THRUST_VERSION
        )
