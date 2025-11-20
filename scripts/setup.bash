#!/bin/bash

wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.0/onnxruntime-linux-x64-1.20.0.tgz
tar -xvzf onnxruntime-linux-x64-1.20.0.tgz
sudo mkdir -p /opt/onnxruntime
sudo cp -r onnxruntime-linux-x64-1.20.0/* /opt/onnxruntime
sudo rm -rf onnxruntime-linux-x64-1.20.0 onnxruntime-linux-x64-1.20.0.tgz