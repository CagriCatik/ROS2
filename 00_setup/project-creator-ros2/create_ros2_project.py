import os
import shutil
import logging

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s]: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

def copy_template_files(root_path):
    setup_logging()
    templates_path = "templates"  # Path to the folder containing template files

    # Define mappings for template files and their destination folders
    file_mappings = {
        "CMakeLists.txt": root_path,
        "empty.world": os.path.join(root_path, "worlds"),
        "empty.yaml": os.path.join(root_path, "config"),
        "LICENSE.md": root_path,
        "package.xml": root_path,
        "robot.urdf.xacro": os.path.join(root_path, "description"),
        "rsp.launch.py": os.path.join(root_path, "launch"),
        "README.md": root_path,
    }

    # Copy template files to their respective folders
    for file, destination in file_mappings.items():
        template_path = os.path.join(templates_path, file)
        destination_path = os.path.join(destination, file)
        os.makedirs(os.path.dirname(destination_path), exist_ok=True)

        try:
            shutil.copy(template_path, destination_path)
            logging.info(f"Successfully copied {file} to {destination}")
        except FileNotFoundError:
            logging.error(f"Template file {file} not found at {template_path}")
        except Exception as e:
            logging.error(f"Error copying {file} to {destination_path}: {e}")

if __name__ == "__main__":
    root_path = "ros2-project"  # Change this to your desired project path
    copy_template_files(root_path)
    logging.info(f"Template files copied to the project folder: {root_path}")
