cmake_minimum_required( VERSION 2.8 )
project( Utils.StructureSensorBot )
project( SceneNavigator.Utils.StructureSensorBot )

find_package( OpenCV REQUIRED )
find_package(Boost COMPONENTS system program_options REQUIRED)
find_package( Curses REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${CURSES_INCLUDE_DIR} )

add_executable( Test.Utils.StructureSensorBot Test.Utils.StructureSensorBot.cpp Utils.StructureSensorBot.cpp)
add_executable( SceneNavigator.Utils.StructureSensorBot SceneNavigator.Utils.StructureSensorBot.cpp Utils.StructureSensorBot.cpp)
add_executable( Mover.Utils.StructureSensorBot Mover.Utils.StructureSensorBot.cpp Utils.StructureSensorBot.cpp)

target_link_libraries( Test.Utils.StructureSensorBot ${OpenCV_LIBS} )
target_link_libraries( Test.Utils.StructureSensorBot ${Boost_SYSTEM_LIBRARY} )
target_link_libraries( Test.Utils.StructureSensorBot ${Boost_PROGRAM_OPTIONS_LIBRARY} )

target_link_libraries( SceneNavigator.Utils.StructureSensorBot ${OpenCV_LIBS} )
target_link_libraries( SceneNavigator.Utils.StructureSensorBot ${Boost_SYSTEM_LIBRARY} )
target_link_libraries( SceneNavigator.Utils.StructureSensorBot ${Boost_PROGRAM_OPTIONS_LIBRARY} )


target_link_libraries( Mover.Utils.StructureSensorBot ${OpenCV_LIBS} )
target_link_libraries( Mover.Utils.StructureSensorBot ${Boost_SYSTEM_LIBRARY} )
target_link_libraries( Mover.Utils.StructureSensorBot ${Boost_PROGRAM_OPTIONS_LIBRARY} )

