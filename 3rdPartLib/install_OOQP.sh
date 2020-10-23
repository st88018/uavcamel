sudo apt-get install gfortran
cd MA27
./configure
make
MA27LIB=${PWD}/src/libma27.a
export MA27LIB
cd ..
cd OOQP
./configure
make
