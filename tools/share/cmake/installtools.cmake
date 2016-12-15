#
#   Windows helper function to resolve dll's for .lib files
#
FUNCTION(FIND_DLLS LIB_LIST DLL_PATH DLL_LIST_OUT)

    LIST(REMOVE_ITEM LIB_LIST debug optimized)

    FOREACH(LIB ${LIB_LIST})
        get_filename_component(LIB_NAME ${LIB} NAME_WE)
        get_filename_component(LIB_PATH ${LIB} PATH)

        STRING(REGEX REPLACE "^lib(.*)" "\\1" LIB_NAME ${LIB_NAME})

        SET(DLL_FILE_NAME ${LIB_NAME}.dll)
        SET(DLL_FILE_NAME2 lib${LIB_NAME}.dll)
        #SET(SEARCH_PATHS "${SEARCH_PATHS} ${LIB_PATH} ${DLL_PATH}")
        LIST(APPEND SEARCH_PATHS ${LIB_PATH} ${DLL_PATH})

        FIND_FILE(
            ${LIB_NAME}
            ${DLL_FILE_NAME}
            ${DLL_FILE_NAME2}
            HINTS
            ${SEARCH_PATHS}
            $ENV{LIBPATH}
            $ENV{PATH}
            $ENV{SYSTEMROOT}/system32
            $ENV{VCINSTALLDIR}/bin
            NO_DEFAULT_PATH
       )

        IF(${LIB_NAME})
            LIST(APPEND DLL_FILE_NAME_LIST ${${LIB_NAME}})
            MESSAGE("Found : " ${LIB_NAME})
        ELSE()
            MESSAGE("not found: " ${LIB_NAME})
            MESSAGE("SEARCHRED: ${SEARCH_PATHS}")
        ENDIF()

    ENDFOREACH()
    IF(NOT "${DLL_FILE_NAME_LIST}" STREQUAL "")
        LIST(REMOVE_DUPLICATES DLL_FILE_NAME_LIST)
    ENDIF()
    SET(${DLL_LIST_OUT} ${DLL_FILE_NAME_LIST} PARENT_SCOPE)
    MESSAGE("FOUND: ${DLL_FILE_NAME_LIST}")
ENDFUNCTION()

#
#   Helper function to resolve symlinks
#
FUNCTION(RESOLVE_SYMLINKS _files real_files)
    SET(_resolvedFiles "")
    FOREACH(_file ${_files})
        get_filename_component(_resolvedFile "${_file}" REALPATH)
        LIST(APPEND _resolvedFiles "${_resolvedFile}")
    ENDFOREACH()
    SET(${real_files} ${_resolvedFiles} PARENT_SCOPE)
ENDFUNCTION()

get_target_property(QT_LIB_PATH Qt5::Core LOCATION)
get_filename_component(QT_LIB_PATH ${QT_LIB_PATH} PATH)
