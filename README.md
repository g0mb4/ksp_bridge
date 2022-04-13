sudo apt install -y libprotobuf-dev

wget <https://sourceforge.net/projects/asio/files/asio/1.22.1%20%28Stable%29/asio-1.22.1.tar.bz2>
tar xf asio-1.22.1.tar.bz2
cd asio-1.22.1
./configure
make
sudo make install

mkdir build; cd build
cmake ..
make
sudo make install
sudo ldconfig
