.PHONY : docker_build
docker_build:
	docker build . -t yasmin

.PHONY : docker_run
docker_run:
	docker run -it --net=host --ipc=host --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority" --entrypoint /bin/bash yasmin