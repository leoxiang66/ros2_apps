#!/bin/bash
echo "Start running ..."

# Source the ROS 2 workspace
source ./install/local_setup.bash

# Function to list ROS 2 packages in the src directory
list_packages() {
    local src_path="$1"
    local packages=()
    for d in "$src_path"/*; do
        if [ -d "$d" ]; then
            packages+=("$(basename "$d")")
        fi
    done
    echo "${packages[@]}"
}

# Function to list nodes in a given package
list_nodes() {
    local package_path="$1"
    local package_name="$2"
    local nodes=()

    if [ -d "./install/$package_name/lib/$package_name" ]; then
        for file in $(find "./install/$package_name/lib/$package_name" -type f); do
            nodes+=("$(basename "$file")")
        done
    fi

    echo "${nodes[@]}"
}

# Function to select an option from a list using 'select'
select_option() {
    local prompt="$1"
    shift
    local options=("$@")

    PS3="$prompt "
    select opt in "${options[@]}"; do
        if [ -n "$opt" ]; then
            echo "$opt"
            break
        else
            echo "Invalid option. Try again."
        fi
    done
}

main() {
    local src_path="$(pwd)/src"
    local packages
    packages=$(list_packages "$src_path")

    if [ -z "$packages" ]; then
        echo "No ROS 2 packages found in the src directory."
        exit 1
    fi

    echo "Select a package:"
    echo
    IFS=' ' read -r -a package_array <<<"$packages"
    selected_package=$(select_option "Select a package:" "${package_array[@]}")
    package_path="$src_path/$selected_package"
    nodes=$(list_nodes "$package_path" "$selected_package")

    if [ -z "$nodes" ]; then
        echo "No nodes found in the package '$selected_package'."
        exit 1
    fi

    echo
    echo "Select a node:"
    IFS=' ' read -r -a node_array <<<"$nodes"
    selected_node=$(select_option "Select a node:" "${node_array[@]}")

    echo
    echo "You selected package: $selected_package"
    echo "You selected node: $selected_node"

    # Command to run the selected node
    command="ros2 run $selected_package $selected_node"
    echo
    echo "Running command: $command"

    # Run the command directly
    $command

    # Check for errors
    command_status=$?
    if [ $command_status -ne 0 ]; then
        echo
        echo "Error: The command exited with status $command_status."
        exit $command_status
    fi
}

main

echo
echo