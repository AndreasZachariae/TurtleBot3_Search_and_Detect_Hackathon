#!/bin/bash

export $(grep -v '^#' .env | xargs)
source /opt/ros/foxy/setup.bash
source install