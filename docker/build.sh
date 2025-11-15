#!/bin/bash
set -e

# 自动切换到 build.sh 所在目录
cd "$(dirname "$0")"

docker build -t mocam_image .
