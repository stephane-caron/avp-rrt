IPYTHON=ipython -i

help:
	@echo "Usage:                                                         "
	@echo "                                                               "
	@echo "    make vip-rrt.py -- run AVP-RRT planner in an ipython shell "
	@echo "                                                               "
	@echo "Plots for the torque-limited pendulum:                         "
	@echo "                                                               "
	@echo "    make 11-05 -- (11, 5)-Nm torque limit                      "
	@echo "    make 11-07 -- (11, 7)-Nm torque limit                      "
	@echo "    make 13-05 -- (13, 5)-Nm torque limit                      "
	@echo "                                                               "
	@echo "Plots for RRT parameter identification:                        "
	@echo "                                                               "
	@echo "    make param-dur -- max-traj-duration                        "
	@echo "    make param-mod -- steer-to-goal-modulo                     "
	@echo "    make param-trj -- traj-per-extension                       "
	@echo "                                                               "
	@echo "Other:                                                         "
	@echo "                                                               "
	@echo "    make clean -- clean temporary files                        "
	@echo "    make help -- this message                                  "
	@echo "                                                               "

vip-rrt.py:
	$(IPYTHON) scripts/single-vip-rrt.py

clean:
	rm -f logs/*.log
	find . -name '*.pyc' -delete

11-05:
	$(IPYTHON) scripts/pretty-plots.py traces/torquelim-11-05

11-07:
	$(IPYTHON) scripts/pretty-plots.py traces/torquelim-11-07

13-05:
	$(IPYTHON) scripts/pretty-plots.py traces/torquelim-13-05

param-dur:
	$(IPYTHON) scripts/pretty-plots.py traces/param_ident/max-traj-duration

param-mod:
	$(IPYTHON) scripts/pretty-plots.py traces/param_ident/steer-to-goal-modulo

param-trj:
	$(IPYTHON) scripts/pretty-plots.py traces/param_ident/traj-per-extension
