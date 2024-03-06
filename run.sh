#!/bin/bash

#xdg-open http://localhost:8080 & MY_UID="$(id -u)" MY_GID="$(id -g)" docker compose up
export MY_UID="$(id -u)" 
export MY_GID="$(id -g)" 

docker compose  \
    --file ./docker/docker-compose.yaml  \
    --env-file .env  \
    up

# docker compose  \
#     --file ./docker/docker-compose-headless.yaml  \
#     --env-file .env  \
#     up

# docker compose  \
#     --file ./docker/docker-compose-demo.yaml  \
#     --env-file .env  \
#     up