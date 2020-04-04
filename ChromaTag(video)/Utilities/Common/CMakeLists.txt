#Writer
add_library(Writer JMD_Utils_Writer.cpp)

#Object
add_library(Object JMD_Utils_Object.cpp)

#Timer
add_library(Timer JMD_Utils_Timer.cpp)

#Options
add_library(Options JMD_Utils_Options.cpp)
target_link_libraries(Options Writer Object)
