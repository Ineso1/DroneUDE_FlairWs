# AeroInes Project

## Overview
This repo includes scripts for building and managing the project environment.Instructions for building the project using the provided script `build_AeroInes.sh`.

## Prerequisites
- Ensure you have the necessary dependencies installed.
- Set the `FLAIR_ROOT` environment variable to the appropriate path.
- Have a Unix-based shell environment.

## Build Instructions
Follow these steps to build the project:

1. **Give Execute Permissions to the Script**  
   Before running the script, ensure it has execute permissions. Run the following command in the terminal:
   ```bash
   chmod +x build_AeroInes.sh
    ```
2. **Execute the Build Script**  
    Run the script to build the project:
    ```bash
   ./build_AeroInes.sh
    ```

## Notes
- The script assumes that the directory structure includes `FlairWs/flair-build` and `FlairWs/flair-src`.

- If the `FLAIR_ROOT` environment variable is not set, the script will exit with an error message.
