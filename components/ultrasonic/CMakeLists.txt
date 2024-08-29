file(GLOB SOURCES *.cpp)

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS "." 
                       REQUIRES driver freertos esp_timer esp_rom soc esp_common)
