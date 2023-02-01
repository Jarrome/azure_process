if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)



add_executable(undistort_cap
    undistort.cpp)
add_executable(read_mkv
    playback.cpp)

add_executable(process
    playback_and_undistort.cpp)



# use libturbojpeg0-dev
target_link_libraries(process PRIVATE
    k4a::k4a
    k4a::k4arecord
    # libjpeg-turbo::libjpeg-turbo
    )

target_link_libraries(undistort_cap PRIVATE
    k4a::k4a
    )
target_link_libraries(read_mkv PRIVATE
    k4a::k4a
    k4a::k4arecord
    )

