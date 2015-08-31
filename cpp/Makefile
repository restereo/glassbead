all: go videowrite

peopleHOG: peopleHOG.c
	g++ -O2 `pkg-config --cflags --libs opencv --libs libv4l2` -lrt peopleHOG.c -o peopleHOG

go: go.c 
	g++ -O0 `pkg-config --cflags --libs opencv --libs libv4l2` -lrt go.c -o go
	
	
videowrite: videowrite.c 
	g++ -O0 `pkg-config --cflags --libs opencv --libs libv4l2` -lrt videowrite.c -o videowrite
	
	
