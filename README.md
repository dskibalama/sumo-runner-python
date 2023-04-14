# sumo-runner-python

This repository contains custom code to generate data using the TraCI python API for SUMO traffic simulator.

Maintainer: Dennis Kibalama kibalama.3@osu.edu

Download the repository by cloning via https using the command below:

```shell
git clone https://github.com/dskibalama/sumo-runner-python.git
```

The sample network file used is larger than 100MB which is beyond the file size limit supported by the free version of GitHUb. I have zipped the network file into a `.zip` file and you have to unzip this file into the appropriate location before running the scripts below. 

On linux run the command
```shell
cd env/traffic_model/reduced_columbus_region
unzip -q reduced_columbus_region.net.zip
```
This should then unzip the file `reduced_columbus_region.net.xml` in the folder `env/traffic_model/reduced_columbus_region`

The scripts can be run with an appropriately configured anaconda/miniconda environment with the name `sumo-env`. To setup the same environment please run the commands below:

```shell 
create --name sumo-env
conda activate sumo-env
conda install scipy numpy matplotlib termcolor jupyter ipympl pyproj rtree
```

NOTE: This has been developed primarily for a linux based environment but will be cofigured to work on macOS and windows environments. 

The `generateRoute.ipynb` notebook uses offline collected GPS data stored in a mat file to extract a SUMO route file that can be used for running a simulation in SUMO

The `tripRunner.ipynb` notebook uses the generated route file to run a SUMO simulation and collect route data. This data can then be used in any other tool that requires route information as input. The current implementation has a generated `traffic_sim_gps.rou.xml` file extracted from route data for Route 19. Although this generated route file doesn't return an error in SUMO, the edges aren't accurately captured and is still a work-in-progress.

* For the current implementation, in order to run different route files, you have to manually change to point to the appropriate `<route file>.rou.xml` inside the `traffic_sim.sumocfg` configuration file. A more modular option is to write a custom configuration file using a function that takes a input the name of the desired `<route file>.rou.xml`
