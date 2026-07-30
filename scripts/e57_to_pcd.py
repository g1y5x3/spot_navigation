import argparse
import os
import open3d as o3d
import pye57
import numpy as np

def convert_e57_to_pcd(input_path):
    # 1. Validate input file
    if not os.path.exists(input_path):
        print(f"Error: The file '{input_path}' does not exist.")
        return

    # 2. Generate output filename by replacing the extension
    base_name, _ = os.path.splitext(input_path)
    output_path = f"{base_name}.pcd"

    print(f"Reading: {input_path}...")
    
    # 3. Read E57 file
    e57_data = pye57.E57(input_path)
    
    # Read Cartesian coordinates from the first scan block
    scan = e57_data.read_scan_raw(0)
    x = scan['cartesianX']
    y = scan['cartesianY']
    z = scan['cartesianZ']

    # 4. Create Open3D PointCloud
    points = np.vstack((x, y, z)).transpose()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    print(f"Original point cloud has {len(pcd.points)} points.")

    # 5. Voxel Downsampling (0.1 meters / 10 centimeters)
    voxel_size = 0.1
    print(f"Downsampling with voxel size: {voxel_size}m...")
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    print(f"Downsampled point cloud has {len(pcd_down.points)} points.")

    # 6. Save downsampled cloud to PCD
    print(f"Saving to: {output_path}...")
    o3d.io.write_point_cloud(output_path, pcd_down)
    print("Conversion complete!")

if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Convert and downsample an E57 point cloud file to PCD format.")
    parser.add_argument("input_file", type=str, help="Path to the input .e57 file")
    
    args = parser.parse_args()
    convert_e57_to_pcd(args.input_file)
