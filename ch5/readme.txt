安装opencv
git clone https://github.com/opencv/opencv.git
mkdir build
cd build
cmake ..
make -j22
sudo make install
sudo ldconfig