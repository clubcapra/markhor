[33mcommit e65a086712288cf9327bacf90f269f38eafc5dc1[m[33m ([m[1;36mHEAD -> [m[1;32mweb_ui_launch_file[m[33m)[m
Author: Kevin <kevinlise@hotmail.com>
Date:   Mon Aug 2 14:23:58 2021 -0400

    Added markhor_web_ui.launch, camera_rearview.launch
    need more work.

[33mcommit cb73a079b7509a5e591209f83ae8a4e1efd1c5e1[m[33m ([m[1;31morigin/master[m[33m, [m[1;31morigin/HEAD[m[33m, [m[1;32mmaster[m[33m)[m
Merge: aa93213 7a16253
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Jun 19 13:54:16 2021 -0400

    Merge pull request #12 from clubcapra/markhor_slam
    
    Markhor slam

[33mcommit 7a1625304d06cb8ca0e30abe55e83b9d99bfc7b5[m
Author: icesentry <c.giguere42@gmail.com>
Date:   Sat Jun 12 17:10:41 2021 -0400

    more PR fixes

[33mcommit 7dd3019a13739f3aeedf103b6cf37cb964993c0b[m
Author: icesentry <c.giguere42@gmail.com>
Date:   Sat Jun 12 14:09:53 2021 -0400

    PR fixes

[33mcommit c3b8210c6fb1f837332df0205700a178620a28d5[m
Author: cgiguere <c.giguere42@gmail.com>
Date:   Sat May 22 16:29:27 2021 -0400

    remove unnecessary stuff

[33mcommit c49e5f0cdbacd3afe1a6ce1ec1a22cc0e0b2e102[m
Author: cgiguere <c.giguere42@gmail.com>
Date:   Sat May 22 16:27:04 2021 -0400

    clean up comments

[33mcommit 06031a4c969b456d56e79f1028963026187a6fba[m
Author: cgiguere <c.giguere42@gmail.com>
Date:   Sat May 22 16:22:32 2021 -0400

    add working hector_slam launch

[33mcommit 5439a28a58f659ee06b04e1f1b5d1046f0977f19[m
Author: cgiguere <c.giguere42@gmail.com>
Date:   Sat May 22 15:14:32 2021 -0400

    WIP

[33mcommit aa93213576fac24bc1e6025e45727e6061b1404b[m
Merge: cd203b5 c72b761
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat May 8 16:02:48 2021 -0400

    Merge pull request #8 from clubcapra/feature/HWI_for_drive
    
    Feature/hwi for drive

[33mcommit c72b7614a617806498bc427c2c045199d5a246cc[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat May 8 16:00:39 2021 -0400

    Remove commented code

[33mcommit c8acfab26843307ca2d0c7fed660d30e1d0254c3[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat May 8 16:00:20 2021 -0400

    Include ROS header file

[33mcommit cb136472fd14860d748a3d577e9ea34ef1119d4f[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat May 8 15:59:52 2021 -0400

    Remove unnecessary newline

[33mcommit ef0bdd6a843fe5b5480f67b6aa3a17318471b332[m
Merge: 80de188 cd203b5
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 17:39:02 2021 -0400

    Merge branch 'master' into feature/HWI_for_drive

[33mcommit 80de188aac8cd46e71b490b9992db647f5e92411[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 17:33:36 2021 -0400

    Add cmd to control robot with joystick in README

[33mcommit 379fe990b08d0af555dd7ab14086f425e159e8ef[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 17:32:28 2021 -0400

    Add launch file for xbox js to control the robot

[33mcommit cd203b5c421d99f4e3831c2a64c6fc66779c0010[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 16:57:23 2021 -0400

    Update README.md
    
    Add arg for cmd_vel option when launch teleop_twist_keyboard

[33mcommit b34790e8023098f0cba899e03c3e19b133af88b8[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 16:20:57 2021 -0400

    Rename id var & change ptr to unique ptr for TalonSRX

[33mcommit 063f6e38a5767a317decb95a0e606f61cfac4350[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 16:20:23 2021 -0400

    Replace ptr with unique_ptr for TalonSRX variable

[33mcommit 2447db4c2cabfd5f0d913711c6c202c2af0d0fba[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 16:17:17 2021 -0400

    Reformat target link lib catkin_lib

[33mcommit cd75061a80d8e7bee3e8b7bdca922160645814d7[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 16:16:45 2021 -0400

    Remove unnecessary comment

[33mcommit 3d7184ed77b557093ee62accc691633fa8aa807e[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 16:16:20 2021 -0400

    Add Thread to Cmake for compilation

[33mcommit ea2040b7c98d6fcbfa881ad28416d88a58d63ef6[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 15:16:33 2021 -0400

    Add params in makhor_base launch for drive ID

[33mcommit e9ec6ce12b1eb44a04b469297df20171ed521155[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 15:15:54 2021 -0400

    Reorganize lines to follow front->rear order

[33mcommit 84d0cd01cca72b31f787c4e2ded844c34c051d2c[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 15:14:17 2021 -0400

    Add send speed to the drives in the wr func of HWI

[33mcommit 9d5c6d0b27af610e7e52089f8454e14728e6ebe9[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 15:12:41 2021 -0400

    Add comment for the read function in HWI

[33mcommit 914a5f7bdc3ef5193725f389262baa94d3c0a2c0[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 13:00:40 2021 -0400

    Add comment for STD include

[33mcommit 324fd4ece2035f71700f2e8ef0605863ecef5d72[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 13:00:17 2021 -0400

    Add creating of TalonSRX & divided init in functs

[33mcommit e324e9f48906b41c53fa0c7e81be7c23f5f14b78[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 12:58:01 2021 -0400

    Add TalonSRX variables to the HWI class

[33mcommit ffb1b5370f067a6651934a0b201bd0ff2d87a1d4[m
Merge: 8883d2e 4f909d6
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 20 11:28:40 2021 -0400

    Merge pull request #7 from clubcapra/diff_drive
    
    Diff drive implementation

[33mcommit 08ead61389092427b89029dd6c98b9d1455102c1[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:48:47 2021 -0500

    Add .cpp for markhor HWI with funct definition

[33mcommit ff67c257880ddcb914362333e6400fecbcbabc52[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:48:22 2021 -0500

    Change .h to .hpp for include

[33mcommit 6d6e7e7b9af0fc613279c087c1fa228497e88897[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:47:54 2021 -0500

    Rename with .hpp & only put funct decl

[33mcommit 2b291509dbc92594e3871a66bde5d90c7d93fc2b[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:47:17 2021 -0500

    Add a source variable containing .cpp files

[33mcommit 7c08572bbef3cfbc0c877a2636eb1a209142cda1[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:44:08 2021 -0500

    Add include directories for src & include/*

[33mcommit 8bb574c039948cbced8d144bd0041a249a8afec5[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:42:10 2021 -0500

    Add CXX flag and bumper c++ version to 14

[33mcommit 458f2190aed1283fe9237d837090cb4dfdf91fa6[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 18:41:39 2021 -0500

    Remove comment in CMake in markhor_base

[33mcommit 4f909d68f0192ed143e63829dc23a3dfc247bf37[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 17:10:39 2021 -0500

    Remove tracked_vehicule_simple.world

[33mcommit 15e4c1fd0d3696fdf6da40471d489a647be936c8[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 16:37:52 2021 -0500

    Add doc for running sim & flipper control

[33mcommit 3f7510a5d0e505fa237dc69d7929364feedf2e41[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 16:30:09 2021 -0500

    Remove simple tracked vehicule world launch file

[33mcommit 596a54771ef9ae1f3e4cb096f84e100173d174b2[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 16:29:24 2021 -0500

    Apply ROS format to launch files

[33mcommit bbbd0dd37b48d8354fe93dd95be8b16973b84856[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 16:28:26 2021 -0500

    Remove trailling newline

[33mcommit 3dbef9ae04d549da39318a10c46e77cf46222aeb[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Mar 13 16:28:08 2021 -0500

    Apply naming scheme to flipper control file

[33mcommit 71ec6138ae6c789e0a25191b33c7db679e6b3f61[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Jan 31 14:20:20 2021 -0500

    Add tracked vehicule simple world for PoC of the simulation

[33mcommit d91fc92765a6b49e6b24077cb2dbcab88732814e[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Jan 31 14:08:07 2021 -0500

    Remove TX1/RPi from CTRE lib

[33mcommit f4899e5e38a93ab1187c09999de738efb5cf555a[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Jan 24 17:48:16 2021 -0500

    Update CMake minimum version

[33mcommit 08799920240423d7ebcd5cf4d4e1783206a8ee60[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Jan 24 17:23:08 2021 -0500

    Add include for CTRE lib

[33mcommit 74bb49fde941d0c55a2f161ca72b3476c8b6ccd2[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Jan 24 17:22:52 2021 -0500

    Add CTRE lib to CMake

[33mcommit ca1f10304e759a70b1533eeda85a98e512ff1748[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Jan 24 17:19:15 2021 -0500

    Add CTRE lib

[33mcommit 41f7033e9bf9ce8226dc50d0381d58a33ff2c749[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:18:30 2020 -0500

    Add launch file for markhor with husky's wheels

[33mcommit f6e6321298ea95c48ad2aca75eef5468edb66f40[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:17:58 2020 -0500

    Add launch file to spawn markhor URDF with husky's wheels

[33mcommit 9928dd422dc2fcec62e9624811b75163a379608b[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:17:40 2020 -0500

    Add launch file to spawn markhor URDF

[33mcommit 43621f2fdbb3d5e56f78a10de93c9f3dcbc45fe4[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:16:47 2020 -0500

    Add husky's wheel's  xacro

[33mcommit 9946fed029e37e3d5eb102304b9d941417ab4e51[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:15:58 2020 -0500

    Add a markhor URDF with husky wheels

[33mcommit 2155604563be981c49171bf1b940075d60caa430[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:14:03 2020 -0500

    Add husky's wheel mesh

[33mcommit d64d39c8034b664b5a8cd9125a8964bee52d2282[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:10:21 2020 -0500

    Add launch to spawn markhor URDF to follow husky example

[33mcommit 1b8a7077c807f0a53387004fbf3cc52732ad8bc8[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:09:34 2020 -0500

    Use previous configuration for gazebo

[33mcommit 449b08c09cbc875d060608afcfad54a5258e2a98[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:07:27 2020 -0500

    Remove test.launch

[33mcommit 113c53551a3548514dd623a28e956b3c7fa5a704[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:01:11 2020 -0500

    Uncomment libKeysToCmdVelPlugin section

[33mcommit 87b1ccc8f7c37c43d83d4a46faefa9301dd951a1[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 20:00:00 2020 -0500

    Add namespace to the plugin so it can create node with this namepsace

[33mcommit 366296b18f9df635db3f6107c798611afd34cc33[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:57:20 2020 -0500

    Remove since they are unused

[33mcommit b9888adfc00345cd8fc0c16826828256102a2031[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:56:40 2020 -0500

    Remove old launch file that doesn't follow nomenclature

[33mcommit 6025ae8c19eb8bd9c0e9e7a34edff06d2504fd4f[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:55:45 2020 -0500

    Add launch file for each specific mode of transportation

[33mcommit 22c04bbc0158459f8527d5db6fd99e18058f519e[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:47:14 2020 -0500

    Add a config file for test with husky wheel

[33mcommit 44f633d6754849c5729c16e123b47e007c3200c8[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:46:28 2020 -0500

    Update controller config file name

[33mcommit ff92594e6b2c974b5bc8c594b7a9c2ee4b6552a0[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:45:39 2020 -0500

    Use control_track instead of diff_drive to follow husky example

[33mcommit 0be534d0f216c8ab763bc65b2b43fe57fdc99d51[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:39:58 2020 -0500

    Change namespace to robot_namespace to follow husky example

[33mcommit 1c0f1f4534b7d04db709b0bcfbaeea1a5d7354ca[m
Merge: 689acd9 b8d595f
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:35:56 2020 -0500

    Merge branch 'diff_drive' of https://github.com/clubcapra/markhor into diff_drive

[33mcommit 689acd99e776ba5fde2a01a5c0c098f65028f66f[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Dec 20 19:35:50 2020 -0500

    Merge controller spawner of flipper and diff drive controller and add ns argument

[33mcommit b8d595f77ec67d0f1999beecb623806c4f961243[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:58:59 2020 -0400

    Add both controller to the Gazebo launch

[33mcommit b32f018e108ce8b5ec5c3c1c300496e8022da6a6[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:58:00 2020 -0400

    Merge controller spawner of flipper and diff drive controller and add ns argument

[33mcommit 49e6a87a986b0f1f1740078f49e39408fd926051[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:54:12 2020 -0400

    Convert tabs to space and set the default to 4 instead of 2

[33mcommit f7f762350279fab7859f5c8780079e0407f76c0e[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:53:18 2020 -0400

    Add comment for flipper gui with rviz

[33mcommit 30352cf61f9e30887765efe6461a1be7e506ffef[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:52:40 2020 -0400

    Remove controller spawner not to conflict with other ros_controller

[33mcommit 5bcd294d6cc7ec20a7f0d23ff490f54fd3cdfde5[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:50:50 2020 -0400

    Change the controller name

[33mcommit bf7f890440d287f997a3c6785ab5e1f1d371447c[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Oct 25 16:50:16 2020 -0400

    Rename contrller manager node

[33mcommit 3cbc89d1b5dcff3861019067e5882272afb1cd5f[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Sep 27 16:18:14 2020 -0400

    Simple reformating

[33mcommit 40abae36aef2c0ebdff26daaa38778a9ac9e980d[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Sep 27 16:17:30 2020 -0400

    Remove unnecessary declaration of HWI

[33mcommit 30af34526b6e218d07788debc4098f09dc0b7715[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Sep 27 16:15:42 2020 -0400

    Reimplemented the my_robot hardware interface code
    
    Fix the ros_control error while loading the hardware interface.

[33mcommit b401eb8da5cb4134c02c621686a96ff295ce9796[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Sep 27 16:13:01 2020 -0400

    Remove / to follow the CMake documentation + remove newline

[33mcommit f109fc72f5dc2eae27b26f0167952eda7892b372[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Fri Sep 25 07:47:03 2020 -0400

    Testing diff_drive controller in Gazebo

[33mcommit 5bf57be3e002bce8d83bec1b7add9728b0021749[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Mon Sep 14 23:08:56 2020 -0400

    Change the diff_drive config file name

[33mcommit 7c4fd6b073bbdf8b580988f1ea2f0495008b2c69[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Mon Sep 14 23:07:12 2020 -0400

    Removed since rename to diff_drive.yaml and diff_drive_control.yaml

[33mcommit ed390934f770245d77824f92e70e445d348a6f70[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Mon Sep 14 22:57:59 2020 -0400

    Format test.launch,markhor_common.xacro and add comment with diff_drive controller

[33mcommit a890d32521d3fb641405ddf1fee8d2b9862e68c4[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Mon Sep 14 22:52:54 2020 -0400

    Rename config file for diff drive to follow name convention with flipper control

[33mcommit d5f0500845162477d12e587cf113c10146f72f6e[m
Merge: c74f9e2 fdc6ae3
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Sep 12 16:56:47 2020 -0400

    Merge branch 'gazebo' into diff_drive

[33mcommit c74f9e2f90581fa728b3848d31de5be279786343[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Sep 12 15:24:25 2020 -0400

    Update the formating

[33mcommit 8bd68f626b1debf1767de6d99e43ccf46597b40c[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Sep 12 15:04:00 2020 -0400

    Rename the debug output from WRITE to READ

[33mcommit 1f4ec9996d7861521d9c40af640abe8fd1603735[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Sep 12 15:03:15 2020 -0400

    Rename the differential drive controller file name

[33mcommit 2dbcb1506c4ae4bd0a965605829e72a8597fda46[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Sep 12 15:01:12 2020 -0400

    Add the ROS C++ Style Guide clang-format to the repository

[33mcommit 536e4c828b15a3a03618bd4d4cdaf4874f180feb[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Jun 6 15:24:05 2020 -0400

    Change right wheel and left wheel name to be the one of the hardware_interface/VelocityJointInterface joint

[33mcommit 33252ea0c8cf1951a6466130e09fe137b7bde123[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sat Jun 6 15:21:36 2020 -0400

    Change naming scheme from Capra Control to Markhor Control

[33mcommit cd40a0a659890c6c6154789f832a510bdedb858e[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Mar 29 15:40:08 2020 -0400

    Fix namespace error

[33mcommit 7829e810bc4754211831b50a215f8941b5e72e9d[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Mar 29 15:38:25 2020 -0400

    Remove useless comment

[33mcommit dc49a3cc28f7cad486154a198e87ab5c5eba5a9b[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Sun Mar 29 15:35:16 2020 -0400

    Refactor Capra_Control to MarkhowHWInterface

[33mcommit 94c1e6cfd6d8195b66cb2d0c027ad08fdeacaf4b[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Thu Mar 19 15:36:36 2020 -0400

    Remove diff_drive config in flipper.yaml file

[33mcommit bf46bfd9690e2ed5903cd41489fa8dbd88eeb5cb[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Thu Mar 19 15:35:40 2020 -0400

    Port from capra_control

[33mcommit fdc6ae374d03ae0d7f4b0de5e8853605efa8ef63[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Sat Mar 14 14:39:24 2020 -0400

    Add lidar and camera sensors

[33mcommit 8633f3a56821a58e8c1f6663ccbedc37fdad0ed3[m
Author: Ludovic Vanasse <ludovicvanasse@gmail.com>
Date:   Thu Mar 12 16:00:39 2020 -0400

    Add gazebo_ros_control to the dependencies

[33mcommit 439a6382a5e82f1e2b63acb06a545e32b7515e3a[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Thu Mar 12 14:14:50 2020 -0400

    Fix tracks simulation

[33mcommit f445547bf89fe5b6d081eb89341f286cc784055c[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Wed Mar 4 09:33:49 2020 -0500

    Remove some space

[33mcommit dcbc06cf2529ee9528fa1054a794d469ce2e4a18[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Tue Mar 3 13:13:10 2020 -0500

    Change package.xml info and add damping on flipper

[33mcommit 9114b92c829c791d26504fc35d880e6b0cb1f2c3[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Mon Mar 2 14:34:52 2020 -0500

    Add ros_control for the flipper

[33mcommit 5afeee0dd7c8d7228c361fc9164d23417c61112a[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Fri Feb 28 20:39:28 2020 -0500

    Fix urdf and add world to move around

[33mcommit 58dc06f84432da1e9cd2beaa6605a43f114e8a96[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Fri Feb 28 18:20:38 2020 -0500

    Add parameter for tracks seperation

[33mcommit c4574470abf95e1a8916cf8347b5f06b0f2d0dcb[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Fri Feb 28 18:18:14 2020 -0500

    Remove useless header

[33mcommit 74aeada21ea6e4920862580864c935ce2de1b5a2[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Fri Feb 28 18:16:02 2020 -0500

    Clean urdf and remove gearbox

[33mcommit 8104324c5f7d6468843e7f63198bac07e1b383e9[m
Author: Marc-Olivier <islandx424@gmail.com>
Date:   Sun Feb 23 22:20:46 2020 -0500

    archive

[33mcommit 4e9bd79a8b06429251601b3c7e119b9562ba9808[m
Author: Marc-Olivier Belisle <islandx424@gmail.com>
Date:   Mon Feb 17 18:31:10 2020 -0500

    Setup the controller for the simulation

[33mcommit 8883d2e1b9aab18b171bb66ba10a2b9105ad2507[m
Author: Marc-Olivier Belisle <islandx424@gmail.com>
Date:   Sat Feb 15 16:53:18 2020 -0500

    Add markhor_description

[33mcommit 40b6f51d69bb57e23576ace16155073b33269be3[m
Author: Marc-Olivier Belisle <islandx424@gmail.com>
Date:   Sat Feb 15 16:47:52 2020 -0500

    markhor base

[33mcommit 5f9871f1ecc38523fe18da5007f9db0ae5860815[m
Author: Marc-Olivier Belisle <islandx424@gmail.com>
Date:   Sat Feb 15 16:46:57 2020 -0500

    markhov_control

[33mcommit e2cbeea02a1857837b69e55f9f92046b048483b3[m
Author: Marc-Olivier Belisle <islandx424@gmail.com>
Date:   Sat Feb 15 16:40:38 2020 -0500

    Add .gitignore

[33mcommit 84a982eafdbc95e009dfc1465c16e14c5f8a4590[m
Author: Marc-Olivier Belisle <islandx424@gmail.com>
Date:   Sat Feb 15 16:35:11 2020 -0500

    first commit
