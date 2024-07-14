import rclpy
import pandas as pd
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from rclpy.serialization import deserialize_message
import numpy as np

TOPIC_TYPE_MAP = {
    '/dvl/twist': TwistStamped,
    '/imu/data': Imu,
    '/filter/free_acceleration': Vector3Stamped
}

# Transformation matrix to flip the x and y axes of the DVL data
R_dvl = np.array([[0, 1, 0],
                  [1, 0, 0],
                  [0, 0, 1]])

# Combined transformation matrix for the IMU data
R_imu = np.array([[-1, 0, 0],
                  [0, 1, 0],
                  [0, 0, -1]])

def extract_data_from_rosbag(bag_path, topics):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    extracted_data = {topic: [] for topic in topics}
    times = {topic: [] for topic in topics}

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic in topics:
            msg_type = TOPIC_TYPE_MAP[topic]
            msg = deserialize_message(data, msg_type)
            extracted_data[topic].append(msg)
            times[topic].append(timestamp)

    return extracted_data, times

def parse_data(extracted_data, topics):
    parsed_data = {}
    for topic in topics:
        if topic not in extracted_data or len(extracted_data[topic]) == 0:
            print(f"No data found for topic: {topic}")
            continue

        first_msg_type = type(extracted_data[topic][0])
        if isinstance(extracted_data[topic][0], TwistStamped):  # Example for TwistStamped
            linear = np.array([[msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] for msg in extracted_data[topic]])
            transformed_linear = linear @ R_dvl.T
            parsed_data[topic] = {
                'linear_x': transformed_linear[:, 0],
                'linear_y': transformed_linear[:, 1],
                'linear_z': transformed_linear[:, 2],
                'angular_x': [msg.twist.angular.x for msg in extracted_data[topic]],
                'angular_y': [msg.twist.angular.y for msg in extracted_data[topic]],
                'angular_z': [msg.twist.angular.z for msg in extracted_data[topic]]
            }
        elif isinstance(extracted_data[topic][0], Imu):  # Example for Imu
            linear_acceleration = np.array([[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z] for msg in extracted_data[topic]])
            angular_velocity = np.array([[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z] for msg in extracted_data[topic]])
            transformed_acceleration = linear_acceleration @ R_imu.T
            transformed_angular_velocity = angular_velocity @ R_imu.T
            parsed_data[topic] = {
                'accel_x': transformed_acceleration[:, 0],
                'accel_y': transformed_acceleration[:, 1],
                'accel_z': transformed_acceleration[:, 2],
                'gyro_x': transformed_angular_velocity[:, 0],
                'gyro_y': transformed_angular_velocity[:, 1],
                'gyro_z': transformed_angular_velocity[:, 2]
            }
        elif isinstance(extracted_data[topic][0], Vector3Stamped):  # Example for Vector3Stamped
            vectors = np.array([[msg.vector.x, msg.vector.y, msg.vector.z] for msg in extracted_data[topic]])
            transformed_vectors = vectors @ R_dvl.T
            parsed_data[topic] = {
                'x': transformed_vectors[:, 0],
                'y': transformed_vectors[:, 1],
                'z': transformed_vectors[:, 2]
            }
        else:
            print(f"Unhandled message type for topic: {topic}, type: {first_msg_type}")
    return parsed_data

def plot_data(parsed_data, times, topics):
    for topic in topics:
        if topic not in parsed_data:
            continue

        data = parsed_data[topic]
        time = times[topic]

        plt.figure()
        if 'linear_x' in data:
            plt.subplot(3, 1, 1)
            plt.plot(time, data['linear_x'], label=f'{topic} linear x')
            plt.plot(time, data['linear_y'], label=f'{topic} linear y')
            plt.plot(time, data['linear_z'], label=f'{topic} linear z')
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(time, data['angular_x'], label=f'{topic} angular x')
            plt.plot(time, data['angular_y'], label=f'{topic} angular y')
            plt.plot(time, data['angular_z'], label=f'{topic} angular z')
            plt.legend()
        elif 'accel_x' in data:
            plt.subplot(3, 1, 1)
            plt.plot(time, data['accel_x'], label=f'{topic} accel x')
            plt.plot(time, data['accel_y'], label=f'{topic} accel y')
            plt.plot(time, data['accel_z'], label=f'{topic} accel z')
            plt.legend()

            plt.subplot(3, 1, 2)
            plt.plot(time, data['gyro_x'], label=f'{topic} gyro x')
            plt.plot(time, data['gyro_y'], label=f'{topic} gyro y')
            plt.plot(time, data['gyro_z'], label=f'{topic} gyro z')
            plt.legend()
        elif 'x' in data:
            plt.subplot(3, 1, 1)
            plt.plot(time, data['x'], label=f'{topic} x')
            plt.plot(time, data['y'], label=f'{topic} y')
            plt.plot(time, data['z'], label=f'{topic} z')
            plt.legend()

        plt.xlabel('Time')
        plt.show()

def main():
    bag_path = '/home/mk/SAUV/SAUV-Autonomy/rosbag2_2024_07_08_Y_AXIS'
    topics = ['/dvl/twist', '/imu/data', '/filter/free_acceleration']

    extracted_data, times = extract_data_from_rosbag(bag_path, topics)
    parsed_data = parse_data(extracted_data, topics)
    plot_data(parsed_data, times, topics)

if __name__ == '__main__':
    main()
