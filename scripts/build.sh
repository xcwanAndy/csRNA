sudo apt-get install mercurial scons swig gcc m4 python python-dev libgoogle-perftools-dev g++ libprotobuf-dev -y
sudo apt-get install build-essential -y
pip3 install six

if [ ! -d ../tests/test-progs/hangu-rnic/bin ];then
	mkdir ../tests/test-progs/hangu-rnic/bin
fi

if [ ! -d res_out ];then
	mkdir res_out
fi
