#!/bin/bash

ROOT=$(dirname "$(readlink -f "$0")")/..
cd $ROOT

sudo docker build -f docker/Dockerfile -t devine .
