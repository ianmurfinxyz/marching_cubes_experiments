test : main.c clock.c clock.h metaballs.c metaballs.h
	gcc -o test main.c clock.c metaballs.c -lSDL2 -lGLU -lGLX_mesa -lm
