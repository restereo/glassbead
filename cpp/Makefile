all: go videowrite

camtest: camtest.c
	g++ -O2 `pkg-config --cflags --libs opencv --libs libv4l2` -lrt camtest.c -o camtest

go: go.c 
	g++ -O0 `pkg-config --cflags --libs opencv --libs libv4l2` -lrt go.c -o go
	
	
videowrite: videowrite.c 
	g++ -O0 `pkg-config --cflags --libs opencv --libs libv4l2` -lrt videowrite.c -o videowrite
	
	
