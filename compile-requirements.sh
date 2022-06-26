#!/bin/bash

set -o errexit
set -o nounset

original_path="$PWD"

cd "$(dirname "${BASH_SOURCE[0]}")"

# See https://github.com/jazzband/pip-tools/
pip-compile requirements.in

cd "$original_path"
