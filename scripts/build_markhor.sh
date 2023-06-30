#! /bin/bash

# Script that builds the Markhor image
docker build -t ghcr.io/clubcapra/markhor/markhor:latest -f Dockerfile.markhor .