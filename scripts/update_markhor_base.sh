#! /bin/bash

docker build -t capraets/markhor-base-ros1 -f Dockerfile.base .
docker push capraets/markhor-base-ros1