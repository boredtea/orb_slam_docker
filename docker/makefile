.PHONY: default
default: images

.PHONY: images
images: ros2-base orb_slam3

.PHONY: ros2-base
ros2-base: ros2-base.Dockerfile
	docker build -f ros2-base.Dockerfile -t ros2-base:humble ..

.PHONY: orb_slam3
orb_slam3: ros2-base orb_slam3.Dockerfile
	docker build -f orb_slam3.Dockerfile -t orb_slam3:humble ..

.PHONY: container
container: orb_slam3
	docker run --network host -it --rm --name orb_slam3 orb_slam3:humble