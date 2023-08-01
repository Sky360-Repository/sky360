#!/usr/bin/env bash
version=1.0.4
docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t sky360/ros2-dev-humble:$version -t sky360/ros2-dev-humble:latest --target ros2-dev