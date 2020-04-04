#dependencies
if(OpenCV_FOUND)
    set(Dependencies ${Dependencies} ${OpenCV_LIBS})
endif(OpenCV_FOUND)

#common
add_library(ChromaTag_Common JMD_ChromaTag_Common.cpp)

#detect
add_library(ChromaTag_Detect JMD_ChromaTag_Detect.cpp)
target_link_libraries(ChromaTag_Detect Math Vision_Process Vision_Point Vision_Line ChromaTag_Common ${Dependencies})

#decode
add_library(ChromaTag_Decode JMD_ChromaTag_Decode.cpp)
target_link_libraries(ChromaTag_Decode ChromaTag_Common ${Dependencies})

#pose
add_library(ChromaTag_Pose JMD_ChromaTag_Pose.cpp)
target_link_libraries(ChromaTag_Pose ChromaTag_Common ${Dependencies})

#add ChromaTag
add_library(JMD_ChromaTag JMD_ChromaTag.cpp)
target_link_libraries(JMD_ChromaTag Writer Timer ChromaTag_Common ChromaTag_Detect ChromaTag_Decode ChromaTag_Pose ${Dependencies})