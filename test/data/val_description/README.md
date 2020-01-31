# Project Name - val_description
# Maintainer - Jordan Lack - jordan.t.lack@nasa.gov
# Maintainer - Frank Mathis frank.b.mathis@nasa.gov


#### Description
The val_description package contains the files needed to generate an URDF for Valkyrie as well as the meshes for visualization. Also contained in the val_description are xacro files for generating Valkyrie instance files. Valkyrie instance files are xml files that contain information about the hardware(actuators, shared memory nodes, sensors, etc). Some of this information is shared between the URDF and instance files. With this, the various coefficient files are included.

The URDF's are generated when you `catkin_make install` a Catkin workspace that contains val_description. 

#### Credits
- Johnson Space Center - ER4 - Valkyrie Team
- IHMC humanoids group
- Maurice Fallon and his group
- Open Source Robotics Foundation(OSRF)

#### License
- NASA-1.3