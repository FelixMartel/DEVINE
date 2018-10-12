#!/usr/bin/env bash

# Docker image: https://github.com/dldl/sphinx-server
docker run --rm -itd -v "$(pwd)/source":/web -p 8000:8000 --name sphinx-server dldl/sphinx-server
