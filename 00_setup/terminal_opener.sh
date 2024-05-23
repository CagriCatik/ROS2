#!/bin/bash

# Check if the number of windows is provided as a command-line argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <number_of_windows>"
    exit 1
fi

# Number of terminal windows to open
num_windows=$1

# Function to open a terminal window with a given title and command
open_terminal() {
    x-terminal-emulator -T "$1" -e "bash -c '$2; exec bash'"
}

# Loop to open the specified number of terminal windows
for ((i=1; i<=$num_windows; i++)); do
    title="Terminal $i"
    command_to_run="echo This is $title"
    open_terminal "$title" "$command_to_run"
done

# Optional: Add a sleep to keep the script running so the terminals stay open
sleep 1
