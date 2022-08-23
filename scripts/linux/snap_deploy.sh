#!/bin/bash

WORKING_DIR="$1"
docker run -t \
  -v "${WORKING_DIR}":"${WORKING_DIR}" \
  -e LC_ALL="C.UTF-8" \
  -e LANG="C.UTF-8" \
  ubuntu:bionic \
  sh -c "echo 'tzdata tzdata/Areas select Europe' | debconf-set-selections && echo 'tzdata tzdata/Zones/Europe select Paris' | debconf-set-selections && export DEBIAN_FRONTEND=noninteractive && apt-get -qq update && apt-get install -y tzdata && apt-get -y install snapcraft && cd ${WORKING_DIR} && snapcraft && snapcraft push *.snap --release edge"
