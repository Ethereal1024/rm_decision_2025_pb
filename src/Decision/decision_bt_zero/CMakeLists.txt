cmake_minimum_required(VERSION 3.10)
project(decision_bt_zero)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/decision_bt_zero.cpp
        include/decision_bt_zero/decision_bt_zero.hpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
        src/decision_bt_zero.cpp
)
#############
## Testing ##
#############

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
            ament_cmake_copyright
            ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif ()

#############
## Install ##
#############

ament_auto_package(
        INSTALL_TO_SHARE
        config
)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)