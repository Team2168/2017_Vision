BeagleVision - This read me is currently under development. KH 1/5/14
============
code using openCV on a beaglebone for the 2013 FRC competition. This document explains how to develop code for the beaglebone using Ubuntu and Eclipse. If you simply want to download the code from Github and run it on the beaglebone without modification see the "Run Git Code on BeagleBone" section.


NEED TO SETUP BEAGLEBONE FIRST SO WE CAN COPY LIBRARIES FOR IT


##A. Instructions to Pull code from Git/Modify it/Run on Bone

###A.1 Set up IDE
1. Install Ubuntu (http://www.ubuntu.com/download/desktop/create-a-usb-stick-on-windows)
2. Open Terminal in Ubuntu and update all the repos feeds: `sudo apt-get update`
3. Install eclipse: `sudo apt-get install eclipse-platform`
4. After the installation launch eclipse from the terminal using: `eclipse &`
5. Add the CDT plug-in: Go to help->install new software and add the `CDT Main tools` for eclipse (http://www.eclipse.org/cdt/downloads.php)
6. Add the `Egit plugin` to eclipse
7. Add the `Remote Systems End-User Runtime Plugin` under General Tools
8. In eclipse: `File->import->git->Projects from Git->URL`
9. Add URL of repo (https://github.com/Team2168/BeagleVision.git)
10. Import existing projects or select import as general project
11. If needed rightclick on the BeagleVision project and select `New->convert to c/c++ project`
12. The BeagleVision Project should now be in your workspace
13. Ignore Project Errors/Warnings and close Eclipse


###A.2 Compiling Code

#### A.2.1: Install Cross Compiler ToolChain
1. In Ubuntu open terminal and install cross compiler: `sudo apt-get install g++-arm-linux-gnueabi`
2. Install g++: `sudo apt-get install g++`
3. Install git: `sudo apt-get install git`
4. Install cmake: `sudo apt-get install cmake`
5. Install pkg-config: `sudo apt-get install pkg-config-arm-linux-gnueabi`
6. Install GTK: `sudo apt-get install libgtk2.0-dev`
7. Install FFMPEG: ` sudo apt-get install ffmpe

#### A.2.2: Copy Libraries from BeagleBone so we do not need to compile them again
1. In Ubuntu open terminal
2. `sudo scp root@192.168.1.33:/usr/lib/* /usr/arm-linux-gnueabi/lib`

#### A.2.3: Install OpenCV and CrossCompile for the Arm Architecture
1. Open terminal in Ubuntu
2. `cd /home`
3. `sudo mkdir OpenCVArm`
4. `cd OpenCVArm`
5. `sudo git clone https://github.com/Itseez/opencv.git`
6. `cd opencv/platforms/linux`
7. `sudo mkdir -p build_softfp`
8. `cd build_softfp`
9. `sudo cp -a /home/OpenCVArm/opencv/3rdparty/include/ffmpeg_/. /usr/include/`
10. `sudo cp -a /usr/arm-linux-gnueabi/lib/glib-2.0/include/. /usr/include/`
11. `sudo cp -a /usr/arm-linux-gnueabi/lib/gtk-2.0/include/. /usr/include/`
9. `sudo cmake -DSOFTFP=ON -DPKG_CONFIG_EXECUTABLE=/usr/bin/arm-linux-gnueabi-pkg-config -DWITH_GTK=ON -DWITH_FFMPEG=ON -DWITH_LIBV4L=OFF -DWITH_V4L=OFF -DWITH_1394=OFF -DWITH_OPENCL=OFF -DCMAKE_TOOLCHAIN_FILE=../arm-gnueabi.toolchain.cmake ../../..`
12. `sudo make`
13. `sudo make install`
14. `sudo xhost +`
14. Now the arm version of OpenCV is in `/home/OpenCVArm/opencv/platforms/linux/build_softfp/install`
15. To verify you can run `file install/lib/libopencv_core.so.3.0.0` and see it is arm architecture
16. SSH into bone `sudo ssh -x -l root 10.21.68.1.33`
17. Copy ARM OpenCV shared libraries on desktop to bone `sudo scp -r kevin@10.21.68.104:/home/OpenCVArm/opencv/platforms/linux/build_hardfp/install/lib/* /lib`

> note we run the last command from the target to avoid permission issues

<pre>
<b>Note: if you run into an error here where the linker ld can not find a certain library prefaced 
such as /lib/pthread.so</b>

This is because certain shared libraries on the bone were build with hardcoded paths, and have been
hard coded with /lib/ or /usr/lib in their path so the linker is looking for the library in the /lib 
folder, unfortunately this is not where we are keeping our arm versions of the libraries so we need 
to fix this.

Simply do a find for the arm version of the library: <b>sudo find / -name pthread*</b>

This will list all versions of the library, the arm versions will be in the 
<b>/usr/arm-linux-gnueabi/lib/</b> directory

If found, 
use vi or gedit to open the .so file and remove all the prefix from Group() tag

<b>sudo vi /usr/arm-linux-gnueabi/lib/libpthread.so</b>

Change <b>Group(lib/pthread.so.0 /usr/lib/pthread_nonshared.a)</b>
to <b>Group(pthread.so.0 pthread_nonshared.a)</b>

Do this for all such libraries.

I only had to do this for the libpthread.so and libc.so libraries

If the library is not found, you need to transfer it from the bone.
</pre>




#### A.2.3 Install Curl Library and CrossCompile for the Arm Architecture
1. Open terminal in Ubuntu
2. sudo apt-get install autoconf
3. sudo apt-get install libtool
2. cd /home
3. sudo mkdir CurlArm
4. cd CurlArm
5. sudo git clone git://github.com/bagder/curl.git
6. cd curl
7. sudo ./buildconf
8. uname -p (change i686 in next line with output of this line)
8. sudo ./configure --host=arm-linux-gnueabi --build=i686-linux CFLAGS='-Os' --with-ssl=/usr/bin/openssl --enable-smtp
9. sudo make
10. sudo make install
11. now the  Arm version of the curl library is in /usr/local/


#### A.2.4 Setup Eclipse to Cross Compile C++ Project
1. Open Eclipse (~$Eclipse &) and right-click on the BeagleVision project and select properties
2. C/C++ Build->settings (Under Debug Configuration)
3. modify the commands of GCC C++ Compiler, GCC C Compiler, GCC G++ Linker, GCC Assembler to be:
    arm-linux-gnueabi-g++
    arm-linux-gnueabi-gcc
    arm-linux-gnueabi-g++
    arm-linux-gnueabi-as
4. Click apply but do not close window
5. Go to C/C++ General->Paths & Symbols
6. Select GNU C
7. Select the Includes Tab
8. Add the following but before clicking OK, select add to all configurations checkbox for each
9. Add /usr/local/includes
10. Add /home/OpenCVArm/opencv/platforms/linux/build_softfp/install/include/
11. Add /usr/arm-linux-gnueabi/include
12. Add /home/CurlArm/curl/include/curl
12. Select GNU C++
13. Select the Includes Tab
14. Add the following but before clicking OK, select add to all configurations checkbox for each
15. Add /usr/local/includes
16. Add /home/OpenCVArm/opencv/platforms/linux/build_hardfp/install/include/
17. Add /home/CurlArm/curl/include/curl
17. Add /usr/arm-linux-gnueabi/include/c++/4.6.3
18. Select GNU C++->Library Paths Tab
19. Add /home/OpenCVArm/opencv/platforms/linux/build_hardfp/install/lib
20. Add /usr/local/lib
21. Add /usr/arm-linux-gnueabi/lib
22. Select GNU C++->Libries Tab
23. Add opencv_highgui
24. Add opencv_core
25. Add opencv_imgproc
26. Add curl
27. Hit Apply and OK
27. Now the eclipe IDE is setup

#### A.2.5 Build Code for BeagleBone (On Beaglebone)
1. Open a terminal window
2. Open an SSH connection to the bone by typing in ssh 10.21.68.xxx
3. Once connected type in nano build.sh
4. Copy and past the following script, g++ -o main main.cpp -lopencv_highgui -lopencv_core -lcurl -lopencv_imgproc
5. Press CTRL+O to save and exit. Name it as build.sh
6. Make the build script runnable by using the following command, chmod +x build.sh
7. Run the build script by ./build.sh

##Run Code On Beagle Bone

###Set Up Beaglebone
1. Install latest image of angstrom linux from: http://beagleboard.org/latest-images  
  Angstrom-Cloud9-IDE-GNOME-eglibc-ipk-v2012.05-beaglebone-2012.11.22 (came with opencv and curl libraries)
2. Follow instructions at: http://beagleboard.org/static/beaglebone/latest/README.htm
3. Connect to internet - bone has DHCP enabled by default on it's physical ethernet interface
4. opkg update
5. opkg install opencv ( 2.4.2-r0 )
6. opkg install libcurl-dev? ( 7.24.0-r2 )
7. Set static IP. e.g. 10.21.68.xxx  
  http://www.gigamegablog.com/2012/02/06/beaglebone-linux-101-assigning-a-static-ip-address-with-connman/
8. Copy code onto beaglebone
9. Disable some unnecessary services. Run the below commands and reboot. Services will no longer start automatically  
Run 'systemctl disable bone101.service' (without quotes) to kill the node server (occasional 5% CPU hit)  
Run 'systemctl disable cloud9.service' (without quotes)

Tips:  
Use 'top' command to see resource usage  
Use systemctl list-units to see how startup services are configured.  
Create your own service: http://beaglebone.cameon.net/home/autostarting-services  

##Set Up Axis Camera
1. Navigate to 192.168.7.90 and click on setup.
2. Ask Victor or James for username and password to get in.
3. After logged on click on the video tab on the left hand side of the screen.
4. Now click click on Camera Settings.
5. Adjust the contrast, brightness, and exposure level to fit the current lighting conditions.


##Set Up Computer
1. Install putty
2. Install xming
3. 

##Build and run the code:
1. Make build.sh script executable: >chmod +x ./build.sh
2. Run build script:  >./build.sh
3. Run compiled executable: >./main
