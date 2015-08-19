# 0. Building

    catkin_make --force-cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo


# 1. Download and extract the PedestrianDetector Package in the scr-folder of a workspace.

# 2. Setting up MATIO (matlab input/output interface)
	Download it: http://sourceforge.net/projects/matio/

# 3. Execute the following commands in the folder of the downloaded file.
		$ tar zxf matio-X.Y.Z.tar.gz
                $ cd matio-X.Y.Z
                $ ./configure
                $ make
                $ make check
                $ make install

# 4. Edit the src/pedestriandetectorros/CMakeLists.txt to make it point to the right position of libmatio.so
# Edit the subscriber to in PedestrianDetection.cpp to the right topic.
