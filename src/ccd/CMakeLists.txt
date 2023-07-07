set(CCD_INCLUDES
        ccd/ccd.h
        ccd/compiler.h
        ccd/ccd_export.h
        ccd/quat.h
        ccd/vec3.h
        ccd/config.h)

set(CCD_SOURCES
        alloc.h
        ccd.c
        dbg.h
        list.h
        mpr.c
        polytope.c
        polytope.h
        simplex.h
        support.c
        support.h
        vec3.c)
#message(STATUS ${CCD_INCLUDES} ${CCD_SOURCES})
#add_library(libccd ${CCD_INCLUDES} ${CCD_SOURCES})
add_executable(ccd main.cpp ${CCD_INCLUDES} ${CCD_SOURCES})