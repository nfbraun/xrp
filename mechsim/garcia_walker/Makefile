.PHONY: all

all: odetest pendulum walker

walker: walker.c
	gcc -o walker -lgsl -lblas -Wall walker.c

test.dat: walker
	./walker > test.dat

pendulum: pendulum.c
	gcc -o pendulum -lgsl -lblas pendulum.c

odetest: odetest.c
	gcc -o odetest -lgsl -lblas odetest.c

