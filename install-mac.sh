brew install openssl libuv cmake
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
export OPENSSL_ROOT_DIR=/usr/local/opt/openssl
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
