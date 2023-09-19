build-boblib-app:
	docker buildx build \
	--platform linux/amd64,linux/arm64 \
	--push \
	-t ghcr.io/bobcamera/bobcamera/boblib-app:$VERSION \
	-t ghcr.io/bobcamera/bobcamera/boblib-app:latest \
	--target boblib-app \
	.
build-ros2-dev:
	docker buildx build \
	--platform linux/amd64,linux/arm64 \
	--push \
	-t ghcr.io/bobcamera/bobcamera/bob-ros2-dev:$VERSION \
	-t ghcr.io/bobcamera/bobcamera/bob-ros2-dev:latest \
	--target bob-ros2-dev \
	.
