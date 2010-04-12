.PHONY: sim

sim: lagrange.dat opendyn.dat

lagrange.dat: lagrange
	./lagrange > lagrange.dat

opendyn.dat: opendyn.py
	./opendyn.py > opendyn.dat

lagrange: lagrange.c
	gcc -o lagrange -lgsl -lblas lagrange.c
