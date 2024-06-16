#!/bin/bash

#xdg-open http://localhost:8080 & MY_UID="$(id -u)" MY_GID="$(id -g)" docker compose up
export MY_UID="$(id -u)" 
export MY_GID="$(id -g)" 

cp $1 ./temp_config.yaml

while :; do 
    read -p "Select d(efault) or r(with rstudio) to specify which compose option to run? " runOption
    response_lower=$(echo "$runOption" | tr '[:upper:]' '[:lower:]')
    if [ "$response_lower" = "default" ] || [ "$response_lower" = "d" ]; then
        echo "Running docker compose using Default option"
        docker compose --file ./docker/docker-compose.yaml up  
            #--env-file .env  \
            #up
        break
    elif [ "$response_lower" = "with rstudio" ] || [ "$response_lower" = "e" ]; then
        echo "Running docker compose using Excl rStudio option"
        docker compose  \
            --file ./docker/docker-compose-rstudio.yaml  \
            --env-file .env  \
            up
        break
    else
        echo "Invalid response. Please enter 'd(efault)', 't(est)' or 'h(eadless)'."
    fi
done

echo ""
