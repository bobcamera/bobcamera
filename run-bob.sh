#!/bin/bash

xdg-open http://localhost:8080 & MY_UID="$(id -u)" MY_GID="$(id -g)" docker compose up