LIBNAME = rospub_core
EXTRA_OBJ = rospub.o visualization.o #sensors.o basemovement.o   # motioncontrol.o
EXTRA_CFLAGS += -DNDEBUG
include ../../Makefile.inc
CXXFLAGS += -std=c++11 -O2
INCLUDES += -I${ROS_DIR}/include
#LDFLAGS += -lcv_bridge -ltf -ltf2_ros -lactionlib -L${ROS_DIR}/lib -lroscpp -lpthread -L${OPENCV_DIR}/ -lboost_signals -lboost_filesystem -lboost_system -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -llog4cxx -lboost_regex -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lboost_chrono -lboost_date_time -lboost_atomic -lconsole_bridge ${OPENCV_DIR}/libopencv_core.so ${OPENCV_DIR}/libopencv_highgui.so ${OPENCV_DIR}/libopencv_imgproc.so


LDFLAGS += -ltf -ltf2_ros -lactionlib -L${ROS_DIR}/lib -lroscpp -lpthread -lboost_regex -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -llog4cxx -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common  -lconsole_bridge
