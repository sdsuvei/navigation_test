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

	- Check the src/pedestriandetectorros/CMakeLists.txt to make it point to the right position of libmatio.so
	- If you get an error like: "undefined reference to H5...".:
		- Delete the installation folder matio-X.Y.Z by "rm -R matio-X.Y.Z"
		- $ tar zxf matio-X.Y.Z.tar.gz
                - $ cd matio-X.Y.Z
                - $ ./configure --without-hdf5
                - $ make
		- $ sudo make install
