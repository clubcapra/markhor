#! /bin/bash

# Script that builds the Markhor base image (Dependencies and files that doesn't change often)
docker build -t ghcr.io/clubcapra/markhor/base:latest -f Dockerfile.base .