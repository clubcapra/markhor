# markhor_slam

This is used to launch the hector_slam package with our configuration

## TF remapping

Since we don't have odometry we need to fake it by using the TF and converting it.

See <https://github.com/DaikiMaekawa/hector_slam_example/blob/master/launch/hector_hokuyo.launch> for the source of the remapping
