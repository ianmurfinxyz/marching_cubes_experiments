test : main.c clock.c clock.h 
	gcc -g -o test main.c clock.c -lSDL2 -lGLU -lGLX_mesa 
