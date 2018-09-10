#!/bin/bash

tmp=$(mktemp -d)
cd $tmp
git clone --recursive git@github.com:projetdevine/guesswhat.git $tmp
sudo docker build -t guesswhat .
rm -rf $tmp

tmp=$(mktemp -d)
cd $tmp
sudo docker build -f docker/base/Dockerfile -t devine-base .
rm -rf $tmp
