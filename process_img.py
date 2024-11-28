import os
from pathlib import Path
import numpy as np
import cv2
from pyproj import Proj, Transformer
from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage

# Initialize coordinate transformation
proj_UTMK = Proj('epsg:5178')
proj_WGS84 = Proj('epsg:4326')
transformer = Transformer.from_proj(proj_WGS84, proj_UTMK)
initpt = np.array([988118.672713562, 1818806.4875518726])  # Origin of campus

def write_gps(msg, f_gps):
    lat = msg.latitude
    lon = msg.longitude
    tnow = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    x, y = transformer.transform(lat, lon)
    f_gps.write(f'{x - initpt[0]:.6f}, {y - initpt[1]:.6f}, {tnow:.6f}\n')

def write_images(msg, folder, imgno):
    tnow = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    cv_img = message_to_cvimage(msg)

    # Undistort
    DIM = (640, 512)
    K_1 = np.array([[445.34, 0., 310.74], [0., 446.40, 249.54], [0., 0., 1.]])
    D_1 = np.array([[0.24, -2.90, 8.05, -7.36]])
    map1_1, map2_1 = cv2.fisheye.initUndistortRectifyMap(
        K_1, D_1, np.eye(3), K_1, DIM, cv2.CV_16SC2)
    undistorted_img = cv_img
    if map1_1 is not None and map2_1 is not None:
        undistorted_img = cv2.remap(
            cv_img, map1_1, map2_1, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # Save
    cv2.imwrite(os.path.join(folder, f'{tnow:.6f}.png'), undistorted_img)

def main(bag_path):
    folder = f'img_{os.path.splitext(os.path.basename(bag_path))[0]}_ther/'
    os.makedirs(folder, exist_ok=True)

    bag_path_obj = Path(bag_path)  # Convert string path to Path object

    with open(os.path.join(folder, 'gpslist.txt'), 'w') as f_gps, AnyReader([bag_path_obj]) as reader:
        connections = [x for x in reader.connections if x.topic in ['/gps', '/thermal/image_raw']]
        imgno = 0

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if connection.topic == '/gps':
                msg = reader.deserialize(rawdata, connection.msgtype)
                write_gps(msg, f_gps)
            elif connection.topic == '/thermal/image_raw':
                imgno += 1
                if imgno % 10 == 0:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    write_images(msg, folder, imgno)

    print(f'Saved {imgno // 10} images to {folder}')

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: python process_img.py <path_to_bag_file>")
    else:
        main(sys.argv[1])

