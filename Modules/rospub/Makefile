LIBNAME = rospub
EXTRA_OBJ = rospub.o motioncontrol.o visualization.o sensors.o
EXTRA_CFLAGS += -DNDEBUG
include ../../Makefile.inc
CXXFLAGS += -std=c++11 -O2

# INCLUDES += -I/opt/ros/kinetic/include
INCLUDES += -I${ROS_DIR}/include

#LDFLAGS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lcv_bridge -ltf -ltf2_ros -lactionlib -L/opt/ros/kinetic/lib -lroscpp -lpthread -L/usr/lib/x86_64-linux-gnu/ -lboost_signals -lboost_filesystem -lboost_system -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -llog4cxx -lboost_regex -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lboost_chrono -lboost_date_time -lboost_atomic -lconsole_bridge

#OPENCVDIR=/usr/lib/x86_64-linux-gnu
# LDFLAGS += -lcv_bridge -ltf -ltf2_ros -lactionlib -L/opt/ros/kinetic/lib -lroscpp -lpthread -L/usr/lib/x86_64-linux-gnu/ -lboost_signals -lboost_filesystem -lboost_system -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -llog4cxx -lboost_regex -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lboost_chrono -lboost_date_time -lboost_atomic -lconsole_bridge ${OPENCVDIR}/libopencv_core.so ${OPENCVDIR}/libopencv_highgui.so ${OPENCVDIR}/libopencv_imgproc.so

LDFLAGS += -lcv_bridge -ltf -ltf2_ros -lactionlib -L${ROS_DIR}/lib -lroscpp -lpthread -L${OPENCV_DIR}/ -lboost_signals -lboost_filesystem -lboost_system -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -llog4cxx -lboost_regex -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lroscpp_serialization -lrostime -lxmlrpcpp -lcpp_common -lboost_chrono -lboost_date_time -lboost_atomic -lconsole_bridge ${OPENCV_DIR}/libopencv_core.so ${OPENCV_DIR}/libopencv_highgui.so ${OPENCV_DIR}/libopencv_imgproc.so
