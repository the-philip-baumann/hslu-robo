# IVIZ 

This applications allows to visualize robots and certain ROS topics on mobile devices. The source code is open-source and can be found [here](https://github.com/KIT-ISAS/iviz).

You have two options to use IVIZ: use a tablet provided by your teacher or install it on your mobile device (only Android supported).

## Installation on Android Device

1. Download the [.apk](https://github.com/KIT-ISAS/iviz/blob/devel/iviz/Binaries/android-7.11.21.apk) file from the devel branch of the git repo.
1. Install the application on your device. Make sure that your device allows installation of apks from the given source.


## Configuration
1. Start a duckiebot.
2. With your mobile device, connect to the network `robo-duckie` and open IVIZ.
3. In IViz, click on 'Connection' and set

    |  |  | |
    |---|---|---|
    |**Master URI** |`http://192.168.1.201:11311` | IP of your duckie (see table below) and port 11311
    | **My URI** | `http://192.168.1.46:5678/` | IP and random port of your mobile device running Iviz.


    The duckiebots have the following IP addresses:

    || |
    |-|-|
    | alpha | 192.168.1.101
    | beta  | 192.168.1.102
    | gamma  | 192.168.1.103
    | delta  | 192.168.1.104
    | epsilon  | 192.168.1.105
    | lamda  | 192.168.1.106
    | omega  | 192.168.1.107
    | zeta  | 192.168.1.108
    | theta  | 192.168.1.109
    | kappa  | 192.168.1.110

    To get the IP of your device, have a look at this [website](https://help.simpletelly.com/article/329-how-to-find-your-android-device-ip-address).

4. In IVIZ, go to *System* > *Aliases* and set the alias `alpha.local` to `192.168.1.101` if you have the `alpha`-duckie.

4. Click on connect.

5. You should see some arrows that correspond to the different parts of your duckiebot. Do the virtual wheels turn if you turn the wheels of your duckiebot?

5. You should also be able to stream the video if you go to *Topic* and click on `/alpha/camera_node/image/compressed`. 