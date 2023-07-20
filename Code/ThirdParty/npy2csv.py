import numpy as np
import csv

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

if __name__ == "__main__":
    npy_file_path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/unsorted data/results/depth_test1/motion_data2.npy"
    csv_file_path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/test.csv"
    
    npy_to_csv(npy_file_path, csv_file_path)
