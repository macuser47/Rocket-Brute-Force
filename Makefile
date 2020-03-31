all: sim
.PHONY: clean sim

sim:
	python3 rocket_sim.py

clean:
	rm -f *.png
	rm -f *.rpth
