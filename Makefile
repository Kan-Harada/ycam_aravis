#
# make PSLIB="phase shift Library Header PATH"
#

.PHONY : clean

CC=g++

CFLAGS=-std=c++11 -I/usr/local/include/opencv -I/usr/local/include/aravis-0.6/ -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/opt/ros/kinetic/include/opencv-3.3.1-dev -O1
OPENCV_LIBS=-L /opt/ros/kinetic/lib/x86_64-linux-gnu -lopencv_stitching3 -lopencv_superres3 -lopencv_videostab3 -lopencv_aruco3 -lopencv_bgsegm3 -lopencv_bioinspired3 -lopencv_ccalib3 -lopencv_cvv3 -lopencv_dpm3 -lopencv_face3 -lopencv_photo3 -lopencv_fuzzy3 -lopencv_hdf3 -lopencv_img_hash3 -lopencv_line_descriptor3 -lopencv_optflow3 -lopencv_reg3 -lopencv_rgbd3 -lopencv_saliency3 -lopencv_stereo3 -lopencv_structured_light3 -lopencv_viz3 -lopencv_phase_unwrapping3 -lopencv_surface_matching3 -lopencv_tracking3 -lopencv_datasets3 -lopencv_text3 -lopencv_dnn3 -lopencv_plot3 -lopencv_xfeatures2d3 -lopencv_shape3 -lopencv_video3 -lopencv_ml3 -lopencv_ximgproc3 -lopencv_calib3d3 -lopencv_features2d3 -lopencv_highgui3 -lopencv_videoio3 -lopencv_flann3 -lopencv_xobjdetect3 -lopencv_imgcodecs3 -lopencv_objdetect3 -lopencv_xphoto3 -lopencv_imgproc3 -lopencv_core3
LDFLAGS=-L. -L/usr/local/lib -laravis-0.6 $(OPENCV_LIBS) -lglib-2.0 -lgobject-2.0 -lpng -lpthread

SRC=$(wildcard *.cpp)
OBJ=$(SRC:%.cpp=%.o)

$(TG) : $(TG).o
	$(CC) -o $@ $^ -std=c++11 $(CFLAGS) $(LDFLAGS)

.cpp.o:
	$(CC) -c $(CFLAGS) -fPIC -o$@ $<
	
.c.o:
	$(CC) -c $(CFLAGS) -fPIC -o$@ $<

clean :
	rm -f $(TG) $(OBJ)  

