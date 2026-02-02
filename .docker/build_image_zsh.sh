
echo -e "Building image ur_jazzy:latest"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target zsh \
--tag ur_jazzy:latest .