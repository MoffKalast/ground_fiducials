# ground_fiducials

A ground based fiducial waypoint generator.

On the robot:

    roslaunch ground_fiducials ground_fiducials.launch

On the workstation:

    roslaunch magni_viz view_nav.launch
    
### Paramaters

* `GO_fiducials`: The array of fiducials we are following, in order of priority. Default `[fid49, fid51]`.
* `STOP_fiducials`: The array of fiducials at which we stop and require interaction to proceed. Default `[fid50]`.

Requires:

- raspicam_node
- aruco_detect
- move_basic
- magni_description
- magni_viz
