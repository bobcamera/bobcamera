#!/bin/sh

set -o errexit
set -o nounset
IFS=$(printf '\n\t')

# Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
printf '\nDocker installed successfully\n\n'

printf 'Waiting for Docker to start...\n\n'
sleep 5

printf 'Adding user to the docker group...\n\n'
sudo usermod -aG docker $USER

printf 'Installing Docker Compose...\n\n'

# Docker Compose
mkdir -p ~/.docker/cli-plugins/
COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
curl -SL https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
chmod +x ~/.docker/cli-plugins/docker-compose
printf '\nDocker Compose installed successfully\n\n'

docker compose version

#UID=${UID} GID=${GID} docker-compose build
#UID=${UID} GID=${GID} docker-compose up
