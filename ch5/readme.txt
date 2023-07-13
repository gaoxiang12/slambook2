安装opencv
首先安装需要的包
sudo apt-get install libgtk2.0-dev pkg-config
下载相应的opencv
git clone https://github.com/opencv/opencv.git
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON ..
make -j22
sudo make install
sudo ldconfig