
import rosbag
from tqdm import tqdm

input_bag = "1.bag"
output_bag = "1_filtered.bag"

topic_map = {
    "/binocular/left_image/compressed": "/scanner/left_image/compressed",
    "/binocular/right_image/compressed": "/scanner/right_image/compressed",
    "/camera/infra1/image_rect_raw/compressed": "/stereo/infra1/compressed",
    "/camera/infra2/image_rect_raw/compressed": "/stereo/infra2/compressed",
    "/camera/imu": "/stereo/imu",
    "/camera/image/compressed": "/stereo/monocular/compressed",
    "/dvs/events": "/dvs/events",
    "/dvs/image_raw/compressed": "/dvs/image_raw/compressed",
    "/dvs/imu": "/dvs/imu",
    "/dvs_rendering/compressed": "/dvs/event_rendering/compressed",
    "/imu/data": "/imu/data",
    "/imu/mag": "/imu/mag",
    "/pressure_sensor/depth": "/pressure_sensor/depth",
    "/bluerov_image/compressed": "/robotview/monocular",
    "/usb_camera/image/compressed": "/waterview/monocular",
}

with rosbag.Bag(input_bag, 'r') as inbag:
    total_msgs = inbag.get_message_count()

    with rosbag.Bag(output_bag, 'w') as outbag:
        available_topics = set([t for t, _ in inbag.get_type_and_topic_info()[1].items()])

        for topic, msg, t in tqdm(inbag.read_messages(), total=total_msgs):
            if topic not in topic_map:
                continue
            if topic not in available_topics:
                continue
            new_topic = topic_map[topic]
            outbag.write(new_topic, msg, t)

print("Done! New bag saved to:", output_bag)