#!/bin/bash
set -e
exec 3>&1
exec 1>&2
exec 2>&3
export LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 LANGUAGE=en_US.UTF-8

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker