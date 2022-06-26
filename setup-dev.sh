#!/bin/bash

set -o errexit
set -o nounset

original_path="$PWD"

cd "$(dirname "${BASH_SOURCE[0]}")"

expected_python_version="$(cat ".python-version" | cut -f 1 -d " ")"
actual_python_version="$(python3 --version | cut -f 2 -d " ")"
if [[ "$expected_python_version" != "$actual_python_version" ]]; then
  echo "You are using the wrong python version: Expected $expected_python_version, got $actual_python_version"
  exit 1
fi

venv_path='.venv'

function activate_venv() {
    activate_venv_str="source $venv_path/bin/activate"
    echo "activating python virtual environment: $activate_venv_str"
    eval "$activate_venv_str"
    echo "activated python virtual environment"

    echo "upgrading pip"
    pip install --upgrade pip
    echo "upgraded pip"
}

if [ -d ".venv" ]; then
    activate_venv
else
    create_venv_str="python3 -m venv $venv_path"
    echo "creating virtual environment: $create_venv_str"
    eval "$create_venv_str"

    activate_venv

    echo "installing python dev-requirements"
    pip install pip-tools
    pip install -r dev-requirements.txt
    echo "installed python dev-requirements"
fi

cd "$original_path"