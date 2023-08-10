#!/usr/bin/env bash
version=1.0.5
docker buildx build --push --platform linux/amd64,linux/arm64 -f Dockerfile . -t sky360/sky360lib-app:$version -t sky360/sky360lib-app:latest --target sky360lib-app
