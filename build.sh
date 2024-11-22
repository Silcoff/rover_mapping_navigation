docker build --build-arg CACHEBUST=$(date +%s) -f Dockerfile-rover -t rover-image .
