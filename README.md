# Nereo PoliTOcean
The code for the ROV Nereo made by PoliTOcean.
The src folder is the src folder of the ros2 workspace.
We are currently using ROS2 Humble.
Inside the src folder there are two packages:
1. gui_pkg
    This package contains the code for the GUI.
2. nereo_sensors_pkg
   This package contains the code for the C++ nodes, used to communicate with the I2C sensors on the Raspberry Pi
## Unit test usage.
Inside the folder ```unit_tests```, you can find a couple of sub directories containing cmake projects, that perform a simple test printing on the stdout some debugging infos.
### Unit test setup instructions.
1. Inside the desired unit test folder, say ```unit_tests/my_unit_test```, run 
   ```bash
   cmake .
   ```
   and then run
   ```bash
   make
   ```
   then you can run the executable, that will have the same name as the unit test folder:
   ```bash
   ./my_unit_test
   ```
