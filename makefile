iscas:short_path.o
	g++ -std=c++11 short_path.o -o iscas
short_path.o:short_path.cpp
	g++ -std=c++11 -c short_path.cpp

