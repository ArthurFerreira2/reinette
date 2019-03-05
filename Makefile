all:reinette

reinette:reinette.c
	gcc -Wall -O3 -lncurses reinette.c -o reinette

