#!/bin/bash

# Function to check if a command exists
command_exists() {
    command -v "$1" &> /dev/null
}

# Check for CUDA
check_cuda() {
    echo "Checking for CUDA installation..."
    if [ -d "/usr/local/cuda" ]; then
        echo "CUDA is installed at /usr/local/cuda"
    else
        echo "CUDA is not installed."
    fi
}

# Check for nvcc
check_nvcc() {
    echo "Checking for nvcc..."
    if command_exists nvcc; then
        nvcc_version=$(nvcc --version | grep release)
        echo "nvcc is installed: $nvcc_version"
    else
        echo "nvcc is not installed."
    fi
}

# Check for cuDNN
check_cudnn() {
    echo "Checking for cuDNN..."
    if [ -f "/usr/include/cudnn.h" ] || [ -f "/usr/local/cuda/include/cudnn.h" ]; then
        cudnn_version=$(grep CUDNN_MAJOR -A 2 /usr/include/cudnn.h 2>/dev/null | grep "#define" | awk '{print $3}')
        echo "cuDNN is installed: Version $cudnn_version"
    else
        echo "cuDNN is not installed."
    fi
}

# Check for nvidia-smi
check_nvidia_smi() {
    echo "Checking for nvidia-smi..."
    if command_exists nvidia-smi; then
        nvidia_smi_output=$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits)
        echo "nvidia-smi is installed."
        echo "GPU details:"
        echo "$nvidia_smi_output"
    else
        echo "nvidia-smi is not installed."
    fi
}

# Run all checks
check_cuda
check_nvcc
check_cudnn
check_nvidia_smi

echo "All checks completed."