#! /bin/bash

docker build -t capraets/markhor-base-ros1 -f base.Dockerfile .
docker push capraets/markhor-base-ros1