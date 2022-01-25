cmake_minimum_required(VERSION 3.12)
set (CMAKE_CXX_STANDARD 14)
 
project(Open3dDemos)

set(Open3D_DIR "C:/Program Files/Open3D/CMake/")

# Find installed Open3D, this imports
#   Open3D_INCLUDE_DIRS
#   Open3D_LIBRARIES
#   Open3D_LIBRARY_DIRS
#   Open3D_C_FLAGS
#   Open3D_CXX_FLAGS
#   Open3D_EXE_LINKER_FLAGS

if(WIN32)
    find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/CMake)
else()
	find_package(Open3D HINTS REQUIRED)
    #find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)
    list(APPEND Open3D_LIBRARIES dl)
endif()
 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Open3D_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${Open3D_EXE_LINKER_FLAGS}")
 
# Set OS-specific things here
if(WIN32)
        
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /FORCE:MULTIPLE")
    # Filament is built with /MT[d], but by default CMake automatically
    # sets "/MD" and "/MDd". They can't coexist, so we need to replace them.
    string(REPLACE "/MD" "/MT" CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})
    string(REPLACE "/MDd" "/MTd" CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})
    string(REPLACE "/MD" "/MT" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
    string(REPLACE "/MDd" "/MTd" CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS_DEBUG})
   
    message(WIN32 "Windows System")
    if (ENABLE_GUI)
       
       
    endif()
elseif(CYGWIN)
elseif(APPLE)
elseif(UNIX)
	add_definitions(-DUNIX)
	add_compile_options(-Wno-deprecated-declarations)
	add_compile_options(-Wno-unused-result)
    add_definitions(-O3)
endif(WIN32)
 
# Open3D
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
 
    # link_directories must be before add_executable
    link_directories(${Open3D_LIBRARY_DIRS})
 
    add_executable(TestVisualizer src/TestVisualizer.cpp)
    add_executable(Voxelization src/Voxelization.cpp)
    add_executable(ViewPCDMatch src/ViewPCDMatch.cpp)
    add_executable(ViewDistances src/ViewDistances.cpp)
    add_executable(TrimMeshBasedOnPointCloud src/TrimMeshBasedOnPointCloud.cpp)
    add_executable(TriangleMesh src/TriangleMesh.cpp)
    add_executable(DenseSLAMGUI src/DenseSLAMGUI.cpp)
    add_executable(OpenMP src/OpenMP.cpp)
    add_executable(PointCloud src/PointCloud.cpp)
    add_executable(LineSet src/LineSet.cpp)
    add_executable(Octree src/Octree.cpp)
    add_executable(Flann src/Flann.cpp)
    add_executable(DenseSLAM src/DenseSLAM.cpp)
    add_executable(Draw src/Draw.cpp)
    add_executable(GeneralizedICP src/GeneralizedICP.cpp)
    add_executable(RegistrationRANSAC src/RegistrationRANSAC.cpp)
    
      
    
    target_link_libraries(TestVisualizer ${Open3D_LIBRARIES})
    target_link_libraries(Voxelization ${Open3D_LIBRARIES})
    target_link_libraries(ViewPCDMatch ${Open3D_LIBRARIES})
    target_link_libraries(ViewDistances ${Open3D_LIBRARIES})
    target_link_libraries(TrimMeshBasedOnPointCloud ${Open3D_LIBRARIES})
    target_link_libraries(TriangleMesh ${Open3D_LIBRARIES})
    target_link_libraries(DenseSLAMGUI ${Open3D_LIBRARIES})
    target_link_libraries(OpenMP  ${Open3D_LIBRARIES})
    target_link_libraries(PointCloud  ${Open3D_LIBRARIES})
    target_link_libraries(LineSet  ${Open3D_LIBRARIES})
    target_link_libraries(Octree  ${Open3D_LIBRARIES})
    target_link_libraries(Flann  ${Open3D_LIBRARIES})
    target_link_libraries(DenseSLAM  ${Open3D_LIBRARIES} Open3D::3rdparty_threads)
    target_link_libraries(Draw  ${Open3D_LIBRARIES})
    target_link_libraries(GeneralizedICP  ${Open3D_LIBRARIES}) 
    target_link_libraries(RegistrationRANSAC  ${Open3D_LIBRARIES}) 

    target_include_directories(TestVisualizer PUBLIC ${Open3D_INCLUDE_DIRS})
 
    # Hot fix windows dll not found issue, assumming we're using the Release build
    #option(BUILD_SHARED_LIBS "Whether Open3D was build as shared library" OFF)
    
    if(WIN32 AND BUILD_SHARED_LIBS)
        message("Will copy Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET TestVisualizer POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endif()
    
    if(WIN32 AND ENABLE_GUI)
        message("Will copy SDL2.dll to ${CMAKE_CURRENT_BINARY_DIR}/Release")
        add_custom_command(TARGET TestVisualizer POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${CMAKE_INSTALL_PREFIX}/lib/SDL2.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/Release)
    endif()
 
else ()
    message(SEND_ERROR "Open3D not found")
endif ()