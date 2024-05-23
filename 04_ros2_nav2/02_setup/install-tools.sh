#!/bin/bash

# Function to check if a command exists
command_exists() {
  command -v "$1" >/dev/null 2>&1
}



# Install Snap if not already installed
install_snap() {
  if ! command_exists snap; then
    echo "Snap is not installed. Installing Snap..."
    sudo apt update
    sudo apt install snapd
    echo "Snap installation complete."
  fi
}

# Install Visual Studio Code via Snap
install_vscode() {
  echo "Installing Visual Studio Code via Snap..."
  sudo snap install code --classic
  echo "Visual Studio Code installation complete."
}

# Install Visual Studio Code extensions
install_extensions() {
  echo "Installing Visual Studio Code extensions..."
  
  # List installed extensions
  code --list-extensions

  # Install ROS extension
  echo "Installing ROS extension..."
  code --install-extension ms-iot.vscode-ros

  # Install CMake extension
  echo "Installing CMake extension..."
  code --install-extension twxs.cmake

  echo "Visual Studio Code extensions installation complete."
}

# Main function
main() {
  install_snap
  install_vscode
  install_extensions
}

# Execute main function
main
