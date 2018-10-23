#!/usr/bin/env bash

# Docker image: https://github.com/dldl/sphinx-server
# To stop the server, use docker stop sphinx-server
# To start the server, use docker start sphinx-server
# To remove the server, use docker rm -v sphinx-server

if [[ "$OSTYPE" == "linux-gnu" ]]; then
    ROOT=$(dirname "$(readlink -f "$0")")/..
    sudo docker run --rm -it -v "$ROOT/docs/source":/web -p 8000:8000 --name sphinx-server dldl/sphinx-server

else
    sudo docker run --rm -it -v "$(pwd)/source":/web -p 8000:8000 --name sphinx-server dldl/sphinx-server

fi
