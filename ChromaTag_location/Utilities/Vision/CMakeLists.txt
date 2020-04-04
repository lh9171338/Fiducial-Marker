# Vision_Process
add_library(Vision_Process JMD_Vision_Process.cpp)

# Vision_Image
add_library(Vision_Image JMD_Vision_Image.cpp)
target_link_libraries(Vision_Image Writer)

# Vision_Point
add_library(Vision_Point JMD_Vision_Point.cpp)

# Vision_Line
add_library(Vision_Line JMD_Vision_Line.cpp)
target_link_libraries(Vision_Line Vision_Point)
