# corner extraction test
IF(faramotics_FOUND)
    ADD_EXECUTABLE(test_extract_corners test_extract_corners.cpp)
    TARGET_LINK_LIBRARIES(test_extract_corners ${GLUT_glut_LIBRARY} ${pose_state_time_LIBRARIES} ${faramotics_LIBRARIES} ${PROJECT_NAME})
ENDIF(faramotics_FOUND)

# corner aperture test
ADD_EXECUTABLE(test_corner_aperture test_corner_aperture.cpp)
TARGET_LINK_LIBRARIES(test_corner_aperture ${PROJECT_NAME})

# Object Detector test
#ADD_EXECUTABLE(test_laser_detector test_laser_detector.cpp)
#TARGET_LINK_LIBRARIES(test_laser_detector ${PROJECT_NAME})
