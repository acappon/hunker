#!/bin/bash

if [[ "$(uname -s)" == "Linux" ]]; then
    find ~/hunker/sw/hunker -type f -exec dos2unix {} +
else
    echo "This script only works on Linux"
fi
