PYTHON := .venv/bin/python
PROJECT := projects/01-water-tank
PROJECT2 := projects/02-coupled-two-tank
RUN_ENV := PYTHONPATH=$(PROJECT)/src MPLBACKEND=Agg MPLCONFIGDIR=.cache/matplotlib

.PHONY: setup test run clean-results

setup:
	python3 -m venv .venv
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install -r $(PROJECT)/requirements.txt

test:
	$(RUN_ENV) $(PYTHON) -m unittest discover -s $(PROJECT)/tests -v
	PYTHONPATH=$(PROJECT2)/src MPLBACKEND=Agg MPLCONFIGDIR=.cache/matplotlib $(PYTHON) -m unittest discover -s $(PROJECT2)/tests -v

run:
	$(RUN_ENV) $(PYTHON) -m water_tank.tank_simulation
	$(RUN_ENV) $(PYTHON) -m water_tank.feedback_control
	$(RUN_ENV) $(PYTHON) -m water_tank.pi_control
	PYTHONPATH=$(PROJECT2)/src MPLBACKEND=Agg MPLCONFIGDIR=.cache/matplotlib $(PYTHON) -m two_tank.experiments

clean-results:
	find $(PROJECT)/results -type f \( -name '*.png' -o -name '*.txt' \) -delete

