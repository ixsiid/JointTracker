set PLATFORMIO_BUILD_FLAGS="-DSLAVE_ADDRESS=%2%"
pio run -e %1 -t upload
