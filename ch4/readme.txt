因为所使用的代码采用的是模板类的sophus库，因此需要安装fmt。
git clone https://github.com/fmtlib/fmt.git

cd fmt
mkdir build
cd build
cmake ..
make
sudo make install

安装Sophus库
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build
cd build
cmake ..
sudo make install
ldconfig
