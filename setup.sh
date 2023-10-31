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

# Docker Compose - OLD
#COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
#sudo curl -L https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-`uname -s`-`uname -m` > /usr/local/bin/docker-compose
#sudo chmod +x /usr/local/bin/docker-compose
#sudo curl -L https://raw.githubusercontent.com/docker/compose/${COMPOSE_VERSION}/contrib/completion/bash/docker-compose > /etc/bash_completion.d/docker-compose
#printf '\nDocker Compose installed successfully\n\n'
#sudo docker-compose -v

# Docker Compose - NEW
mkdir -p ~/.docker/cli-plugins/
COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
curl -SL https://github.com/docker/compose/releases/download/v2.3.3/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
chmod +x ~/.docker/cli-plugins/docker-compose
printf '\nDocker Compose installed successfully\n\n'
docker compose version

#UID=${UID} GID=${GID} docker-compose build
#UID=${UID} GID=${GID} docker-compose up
