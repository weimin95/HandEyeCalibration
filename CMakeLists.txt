# 设置 CMake 的最低版本
cmake_minimum_required(VERSION 3.10)

# 设置项目名称和版本
project(HandEyeCalibration VERSION 1.0)

if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Supported configurations" FORCE)
    message(STATUS "Allowed configurations: ${CMAKE_CONFIGURATION_TYPES}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 定义第三方库的路径变量
set(OPENCV "D:/Proj/3rdParty/opencv410/build" CACHE PATH "Installation path for OpenCV")
set(EIGEN "D:/Proj/3rdParty/Eigen/eigen3" CACHE PATH "Installation path for Eigen")
set(CERES "D:/Proj/3rdParty/Ceres" CACHE PATH "Installation path for Ceres")

set(THIRD_PARTY_DIR 
    ${OPENCV}
    ${EIGEN}
    ${CERES}
)

# 检查目录是否存在
foreach(THIRD_PARTY ${THIRD_PARTY_DIR})
    if(NOT EXISTS ${THIRD_PARTY})
        message(FATAL_ERROR "Directory '${THIRD_PARTY}' does not exist. Please check")
    endif()
endforeach()

# 设置 DLL 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 设置包含目录
include_directories(include)  # 公开的头文件目录
include_directories(source)   # 内部头文件目录

# 设置源文件
file(GLOB_RECURSE SOURCES "source/*")  # 获取所有源文件
file(GLOB_RECURSE HEADERS "include/*")

message("Build files:")
foreach(header ${HEADERS})
    message("${header}")
endforeach()
foreach(source ${SOURCES})
    message("${source}")
endforeach()

# 添加 DLL 目标
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# 将头文件添加到目标中，以便它们在 IDE 中可见
target_sources(${PROJECT_NAME} PRIVATE source)

# 设置私有头文件目录
target_sources(${PROJECT_NAME} PUBLIC ${HEADERS})

# 设置头文件包含路径
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${OPENCV}/include
    ${EIGEN}
    ${CERES}/include
    ${CERES}/include/ceres/internal/miniglog
)

# 链接第三方库
target_link_libraries(${PROJECT_NAME} PRIVATE 
    $<$<CONFIG:Debug>:${OPENCV}/x64/vc16/lib/opencv_world4100d.lib>    # Debug 下的库
    $<$<CONFIG:Release>:${OPENCV}/x64/vc16/lib/opencv_world4100.lib>   # Release 下的库
    $<$<CONFIG:Debug>:${CERES}/lib/ceres-debug.lib>   # Debug 下的 Ceres 库
    $<$<CONFIG:Release>:${CERES}/lib/ceres.lib>  # Release 下的 Ceres 库
)

# 调试信息
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi /O2")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG")

# 设置 DLL 安装路径
set(INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/HandEyeCalibration")

install(TARGETS ${PROJECT_NAME} 
        RUNTIME DESTINATION ${INSTALL_PATH}/bin/$<CONFIG>
        ARCHIVE DESTINATION ${INSTALL_PATH}/lib/$<CONFIG>
)

# 安装头文件到 Include 文件夹
install(DIRECTORY include/ DESTINATION ${INSTALL_PATH}/include)  # 头文件将放置在 Include 文件夹中

set(SAMPLE_PROJECT Sample)
file(GLOB SAMPLES "sample/*.cpp")
add_executable(${SAMPLE_PROJECT} ${SAMPLES})

# 链接示例应用与 DLL
target_link_libraries(${SAMPLE_PROJECT} PRIVATE ${PROJECT_NAME})

# 设置头文件包含路径
target_include_directories(${SAMPLE_PROJECT} PRIVATE 
    ${OPENCV}/include
    ${EIGEN}
)

# 链接第三方库
target_link_libraries(${SAMPLE_PROJECT} PRIVATE 
    $<$<CONFIG:Debug>:${OPENCV}/x64/vc16/lib/opencv_world4100d.lib>   # Debug 下的库
    $<$<CONFIG:Release>:${OPENCV}/x64/vc16/lib/opencv_world4100.lib>  # Release 下的库
)

# 定义所依赖DLL文件的目录列表
set(THIRD_PARTY_DLLS
    $<$<CONFIG:Debug>:${OPENCV}/x64/vc16/bin/opencv_world4100d.dll>
    $<$<CONFIG:Debug>:${CERES}/bin/ceres-debug.dll>
    $<$<CONFIG:Release>:${OPENCV}/x64/vc16/bin/opencv_world4100.dll>
    $<$<CONFIG:Release>:${CERES}/bin/ceres.dll>
)

# 复制所有目录下的 DLL 文件到安装目录
foreach(DLL_FILE ${THIRD_PARTY_DLLS})
    install(FILES "${DLL_FILE}" DESTINATION "$<TARGET_FILE_DIR:${SAMPLE_PROJECT}>")  # 安装 DLL 到指定目录
endforeach()

# 复制所有目录下的 DLL 文件到运行目录
#foreach(DLL_FILE ${THIRD_PARTY_DLLS})
#    # 使用 add_custom_command 执行 DLL 复制操作
#    add_custom_command(TARGET ${SAMPLE_PROJECT} POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_if_different
#        "${DLL_FILE}" "$<TARGET_FILE_DIR:${SAMPLE_PROJECT}>"
#    )
#endforeach()

# 复制所有目录下的 DLL 文件到安装目录
foreach(DLL_FILE ${THIRD_PARTY_DLLS})
    install(FILES "${DLL_FILE}" DESTINATION ${INSTALL_PATH}/bin/$<CONFIG>)  # 安装 DLL 到指定目录
endforeach()
