#!/bin/bash

set -o errexit
set -o nounset
IFS=$(printf '\n\t')

# Docker

# Check if Docker is installed
if command -v docker &> /dev/null; then
    echo 'Docker is already installed, skipping.'
else
    echo 'Docker is not installed, installing...\n\n'
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    printf '\nDocker installed successfully\n\n'

    printf 'Waiting for Docker to start...\n\n'
    sleep 5

    printf 'Adding user to the docker group...\n\n'
    sudo usermod -aG docker $USER
fi

# Docker Compose

# Check if Docker Compose is installed
if command -v docker compose &> /dev/null; then
    echo 'Docker Compose is already installed, skipping.'
else
    echo 'Docker Compose is not installed, installing...\n\n'
    mkdir -p ~/.docker/cli-plugins/
    COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
    curl -SL https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-linux-x86_64 -o ~/.docker/cli-plugins/docker-compose
    chmod +x ~/.docker/cli-plugins/docker-compose
    printf '\nDocker Compose installed successfully\n\n'
fi

sleep 2

YAML_FILE="${USER}.yaml"
if [ ! -f "$YAML_FILE" ]; then
    cp ex_config_video.yaml "$YAML_FILE"
    echo "$YAML_FILE has been created."
else
    echo "$YAML_FILE already exists."
fi

# Run the config script
# ./config.sh
