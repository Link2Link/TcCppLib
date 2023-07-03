cmake_minimum_required(VERSION 3.2)

project(project_test)
message(STATUS "---------------------------")
message(STATUS "Current : ${PROJECT_NAME}")



SET(TEST_INSTALL_DIR ${PROJECT_TOP_DIR}/bin/test)
include(${PROJECT_TOP_DIR}/cmake/TestDepend.cmake)

enable_testing()

FILE(GLOB TEST_SRC "${PROJECT_SOURCE_DIR}/*.cpp")

foreach(item IN LISTS TEST_SRC)
    get_filename_component(SRCNAME ${item} NAME_WE) #得到文件的名字，不带扩展名
    add_executable(${SRCNAME} ${item} ${SRC})
    target_link_libraries(${SRCNAME} doctest_with_main)
    add_test(
            NAME ${SRCNAME}
            COMMAND $<TARGET_FILE:${SRCNAME}>
    )
    install(TARGETS ${SRCNAME} DESTINATION ${TEST_INSTALL_DIR})
    message(STATUS "Test : ${SRCNAME} <- ${item}")

    SET(SRCNAMELIST "${SRCNAMELIST} ${SRCNAME}")
endforeach(item)

if (ENABLE_LCOV)
    message(STATUS "-- lcov Configure")
    find_program(LCOV_BIN lcov)
    if (NOT LCOV_BIN MATCHES "lcov$")
        message(FATAL_ERROR "lcov required, but not found!")
    endif ()
    if (CMAKE_BUILD_TYPE MATCHES Debug)
        include(${PROJECT_TOP_DIR}/cmake/CodeCoverage.cmake)
        APPEND_COVERAGE_COMPILER_FLAGS()

                setup_target_for_coverage_lcov(
                NAME coverage
                EXECUTABLE ctest test
                BASE_DIRECTORY "${PROJECT_SOURCE_DIR}/build"
                EXCLUDE "${PROJECT_SOURCE_DIR}/build/_deps/*" "/usr/*"
#                DEPENDENCIES ${SRCNAMELIST}
        )

    endif ()

endif ()