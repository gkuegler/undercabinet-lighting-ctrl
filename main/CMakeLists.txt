file(GLOB SOURCES *.cpp)

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ".")
