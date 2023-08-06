import numpy as np
import csv
import os

def npy_to_csv(npy_file_path, csv_file_path):
    # Load the .npy file
    data = np.load(npy_file_path)

    # Write the data to the .csv file in the desired format
    with open(csv_file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['x', 'y', 'z'])  # Write the header
        for row in data:
            for x, y, z in row:
                csv_writer.writerow([x, y, z])

# if __name__ == "__main__":
#     npy_file_path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/close/motion_data_rise.npy"
#     csv_file_path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/close/close.csv"
#     npy_to_csv(npy_file_path, csv_file_path)
#     print("Done!!")



def create_file(file_path):
    with open(file_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['dx', 'dy', 'SAD'])  # Write the header

# Set the paths for the source directory with .npy files and the destination directory for .csv files
source_dir = r"C:\Users\WIN10PRO\Desktop\My Stuff\University\BSC\Y3\RT systems\Real-Time-Systems-Lab\Code\Data\vertical rotation\npy"
destination_dir = r"C:\Users\WIN10PRO\Desktop\My Stuff\University\BSC\Y3\RT systems\Real-Time-Systems-Lab\Code\Data\vertical rotation\csv"

# Get a list of all .npy files in the source directory
npy_files = [file for file in os.listdir(source_dir) if file.endswith('.npy')]

# Create the destination directory if it doesn't exist
if not os.path.exists(destination_dir):
    os.makedirs(destination_dir)

# Loop through each .npy file and create an empty .csv file in the destination directory
for npy_file in npy_files:
    npy_file_name = os.path.splitext(npy_file)[0]  # Get the filename without the extension
    csv_file_name = npy_file_name + '.csv'
    csv_file_path = os.path.join(destination_dir, csv_file_name)

    # Create the empty .csv file if it doesn't exist
    if not os.path.exists(csv_file_path):
        create_file(csv_file_path)

    # Convert and save the data to the .csv file
    npy_file_path = os.path.join(source_dir, npy_file)
    npy_to_csv(npy_file_path, csv_file_path)

print("Conversion completed successfully.")
