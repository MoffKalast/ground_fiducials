# ground_fiducials

A ground based fiducial waypoint generator.

Fiducial pdf pages can be printed using the following command when aruco_detect is installed:
 
    rosrun aruco_detect create_markers.py 100 112 fiducials.pdf
    
The 100 and 112 specify the starting and ending numerical IDs of the markers. They must be printed as close to 14 cm by 14 cm as possible.

### Launching

On the robot:

    roslaunch ground_fiducials ground_fiducials.launch

or

    roslaunch ground_fiducials smooth_fiducials.launch

On the workstation:

    roslaunch magni_viz view_nav.launch
    
### Paramaters

* `GO_fiducials`: The array of fiducials we are following, in order of priority. Default `[49, 51]`.
* `STOP_fiducials`: The array of fiducials at which we stop and require interaction to proceed. Default `[50]`.

Requires:

- raspicam_node
- aruco_detect
- move_basic
- magni_description
- magni_viz
