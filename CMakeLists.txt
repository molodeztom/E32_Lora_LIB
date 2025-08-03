# Get Git commit ID and version information for the library
# First, try to get the most recent tag (for Semantic Versioning)
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE E32_GIT_TAG_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE E32_GIT_TAG_RESULT
)

# Get the full Git description (includes commits since tag and dirty state)
execute_process(
    COMMAND git describe --tags --dirty --always
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE E32_GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Process the version information
if(E32_GIT_TAG_RESULT EQUAL 0)
    # Successfully found a tag - use it for the version number
    # Remove 'v' prefix if present (e.g., v1.2.3 -> 1.2.3)
    string(REGEX REPLACE "^v" "" E32_VERSION_NUMBER "${E32_GIT_TAG_VERSION}")
    message(STATUS "Using E32_Lora_Lib version from Git tag: ${E32_VERSION_NUMBER}")
else()
    # No tag found - use a default version
    set(E32_VERSION_NUMBER "0.5")
    message(STATUS "No Git tag found for E32_Lora_Lib, using default version: ${E32_VERSION_NUMBER}")
endif()

# Register the component with the version information
idf_component_register(SRCS "E32_Lora_Lib.c"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "include")

# Add the Git version and version number as compile definitions
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    E32_APP_VERSION="${E32_GIT_VERSION}"
    E32_APP_VERSION_NUMBER="${E32_VERSION_NUMBER}"
)

message(STATUS "Building E32_Lora_Lib with version: ${E32_VERSION_NUMBER} (${E32_GIT_VERSION})")
