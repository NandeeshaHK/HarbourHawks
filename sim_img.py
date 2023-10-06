import os
import shutil
import time

# Source and destination folder paths
source_folder = "/home/nerdnhk/HarbourHawks/Images/imgs"
destination_folder = "/home/nerdnhk/HarbourHawks/Images/copy_img"

# Create the destination folder if it doesn't exist
if not os.path.exists(destination_folder):
    os.makedirs(destination_folder)

while True:
    # Get a list of all files in the source folder
    files = os.listdir(source_folder)
   
    # Filter and sort the files by creation time
    image_files = [file for file in files if file.lower().endswith((".jpg", ".jpeg", ".png", ".gif"))]
    image_files.sort(key=lambda x: os.path.getctime(os.path.join(source_folder, x)))

    # Copy images to the destination folder in ascending order
    for image_file in image_files:
        source_path = os.path.join(source_folder, image_file)
        destination_path = os.path.join(destination_folder, image_file)
        try:
            shutil.copy2(source_path, destination_path)
            print(f"Copied: {image_file}")
            time.sleep(5)
            os.remove(destination_path)
            print(f"Deleted: {image_file}")
        except Exception as e:
            print(f"Error copying {image_file}: {str(e)}")

    print("Copying completed.")

    # # Delete the copied images from the destination folder
    # for image_file in image_files:
    #     destination_path = os.path.join(destination_folder, image_file)
    #     try:
    #         os.remove(destination_path)
    #         print(f"Deleted: {image_file}")
    #     except Exception as e:
    #         print(f"Error deleting {image_file}: {str(e)}")

    # print("Deletion completed.")

    # Sleep for a specified time (e.g., 1 hour) before repeating the process
    # time.sleep(0)  # Sleep for 1 hour before the next copy process
