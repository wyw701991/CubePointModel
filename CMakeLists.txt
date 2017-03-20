cmake_minimum_required(VERSION 2.8)

#PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CubePointModel_SRCS main.cpp
                   CubePointModel.cpp
                           )

SET(CubePointModel_HEADERS  CubePointModel.h
                      )
ADD_EXECUTABLE(CubePointModel ${CubePointModel_SRCS}
		              ${CubePointModel_HEADERS_MOC}
                          ${CubePointModel_UIS_HEADERS})
TARGET_LINK_LIBRARIES(CubePointModel ${PCL_LIBRARIES}
)

