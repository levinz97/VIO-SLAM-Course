include_directories("/usr/include/eigen3")
add_library(${PROJECT_NAME}_backend
        vertex.cc
        edge.cc
        problem.cc
        )