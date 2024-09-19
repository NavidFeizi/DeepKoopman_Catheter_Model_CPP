add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} 
                        PRIVATE KOOPMANlib 
                        PRIVATE TENDONDRIVENlib)

target_compile_definitions(${PROJECT_NAME} PRIVATE WORKSPACE_DIR="${CMAKE_SOURCE_DIR}")