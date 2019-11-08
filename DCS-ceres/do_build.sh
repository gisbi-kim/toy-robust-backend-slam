cd save
rm -rf *

cd ..
mkdir build
cd build
rm -rf *
cmake ..
make -j8
./main $1 $2 $3

cd ..

cd drawer
./do_plot.sh
cd ..
