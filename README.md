# RIEKER_H6 #
This is a small API for communication with Rieker H6 inclinometers.

The whole API is encapsulated in a single class: Flexbus. See the description 
of this class for more details.

The functions implemented are those described in 
[Flexware User Guide](https://www.riekerinc.com/wp-content/uploads/Documents/H6MM/Flexware_UserGuide.pdf)

## Components ##

 * flex        - the API library
 * sample      - sample program
 * test        - unit tests for each interface function


## Installation ##
While the API is does not have any dependencies, the units tests use the [UTPP framework](https://github.com/neacsum/utpp).

Documentation can be generated using [Doxygen](http://www.stack.nl/~dimitri/doxygen/index.html)

All projects have been tested under Visual Studio 2022.

Copyright (c) 2017-2024 Mircea Neacsu<br/>
Rieker&trade; is a registered trademark of [Rieker Inc](https://riekerinc.com)
