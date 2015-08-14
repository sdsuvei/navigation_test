# navigation_test
-- testing of the Hough Line &amp; simple navigation algorithm

# Setting up MATIO (matlab input/output interface)
	- Download it: http://sourceforge.net/projects/matio/
	- Execute the following commands in the directory of the downloaded file.
		$ tar zxf matio-X.Y.Z.tar.gz
                $ cd matio-X.Y.Z
                $ ./configure
                $ make
                $ make check
                $ make install

	- Edit the src/pedestriandetectorros/CMakeLists.txt to make it point to the right position of libmatio.so
