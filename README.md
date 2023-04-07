# sumo-runner-python

This repository contains custom code to generate data using the TraCI python API for SUMO traffic simulator.

Maintainer: Dennis Kibalama kibalama.3@osu.edu

The scripts can be run with an appropriately configured anaconda/miniconda environment with the name `sumo-env`. To setup the same environment please run the commands below:

```shell 
create --name sumo-env
conda activate sumo-env
conda install scipy numpy matplotlib termcolor jupyter ipympl
```

NOTE: This has been developed primarily for a linux based environment but will be cofigured to work on macOS and windows environments. 
