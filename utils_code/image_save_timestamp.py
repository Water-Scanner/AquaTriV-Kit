#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import glob
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage

SAVE_RATE_HZ = 10.0
SAVE_INTERVAL = 1.0 / SAVE_RATE_HZ

# ========== 配置 ==========
# 设置大目录
ROOT_DIR = "path_to_dir"

# 每个话题对应的子目录
CONFIG = [
    ("/bluerov_image/compressed11",             "bluerov"),
    ("/camera/color/image_raw/compressed",    "color"),
    ("/camera/infra1/image_rect_raw/compressed11", "infra1"),
    ("/camera/infra2/image_rect_raw/compressed11", "infra2"),
]

# ========== 工具函数 ==========
_num_re = re.compile(r"(\d+)\.jpg$", re.IGNORECASE)

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def next_index_in_dir(dirpath):
    files = glob.glob(os.path.join(dirpath, "*.jpg"))
    max_id = 0
    for f in files:
        m = _num_re.search(os.path.basename(f))
        if m:
            try:
                max_id = max(max_id, int(m.group(1)))
            except ValueError:
                pass
    return max_id + 1 if max_id > 0 else 1

def decode_compressed_to_bgr(msg):
    np_arr = np.frombuffer(msg.data, dtype=np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img_bgr

# ========== 订阅保存类 ==========
class TopicSaver(object):
    def __init__(self, topic_name, subdir):
        self.topic = topic_name
        self.dir = os.path.join(ROOT_DIR, subdir)
        ensure_dir(self.dir)
        self.counter = next_index_in_dir(self.dir)
        self.last_save_ts = 0.0
        self.ts_file_path = os.path.join(self.dir, "timestamps.txt")
        self.ts_fh = open(self.ts_file_path, "a", buffering=1)
        self.sub = rospy.Subscriber(self.topic, CompressedImage, self.cb, queue_size=10)

        rospy.loginfo("TopicSaver: %s -> %s (start idx=%d)", self.topic, self.dir, self.counter)

    def cb(self, msg):
        now = rospy.get_time()
        if (now - self.last_save_ts) < SAVE_INTERVAL:
            return
        self.last_save_ts = now

        img = decode_compressed_to_bgr(msg)
        if img is None:
            rospy.logwarn("Failed to decode image from %s", self.topic)
            return

        img_name = f"{self.counter}.jpg"
        img_path = os.path.join(self.dir, img_name)

        cv2.imwrite(img_path, img)

        stamp_sec = msg.header.stamp.to_sec() if msg.header.stamp else now
        self.ts_fh.write(f"{self.counter} {stamp_sec:.9f}\n")

        self.counter += 1

    def close(self):
        try:
            self.ts_fh.close()
        except Exception:
            pass

# ========== 主程序 ==========
def main():
    rospy.init_node("compressed_image_saver_4hz", anonymous=False)

    savers = []
    try:
        for topic, subdir in CONFIG:
            savers.append(TopicSaver(topic, subdir))

        rospy.loginfo("Images will be saved into ROOT_DIR=%s at %.2f Hz", ROOT_DIR, SAVE_RATE_HZ)
        rospy.spin()
    finally:
        for s in savers:
            s.close()

if __name__ == "__main__":
    main()
