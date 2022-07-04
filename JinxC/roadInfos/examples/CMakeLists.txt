
# add_executable(simple_test simpleTest.cpp)
# target_link_libraries(simple_test ${ODRMANGER_LIB})


#add_executable(get_lane_info getLaneInfo.cpp)
#target_link_libraries(get_lane_info ${ODRMANGER_LIB})

# add_executable(frontCar_test isFrontcartest.cpp ../src/posInfo.cpp)
# target_link_libraries(frontCar_test ${ODRMANGER_LIB})



# add_executable(pos_info getPosInfoClassTest.cpp ../src/posInfo.cpp)
# target_link_libraries(pos_info ${ODRMANGER_LIB})


# add_executable(pos_info getPosInfoClassTest.cpp ../src/posInfowh.cpp)
# target_link_libraries(pos_info ${ODRMANGER_LIB})


# add_executable(roadIDTest roadIDTest.cpp)
# target_link_libraries(roadIDTest ${ODRMANGER_LIB})


 add_executable(roadIDTest2 roadIDTest2.cpp ../src/tinyxml2.cpp ../OdrManager/BaseNodes/OdrRoadHeader.hh)

# add_executable(lane_type getLaneType.cpp)
# target_link_libraries(lane_type ${ODRMANGER_LIB})


# add_executable(test test.cpp)
