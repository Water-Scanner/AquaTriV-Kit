import rosbag

input_file = 'input.bag'
output_file = 'output.bag'
start_offset = 920
duration = 240.0

with rosbag.Bag(output_file, 'w') as outbag:
    first_t = None
    for topic, msg, t in rosbag.Bag(input_file).read_messages():
        if first_t is None:
            first_t = t
        
        rel_time = (t - first_t).to_sec()
        
        if rel_time >= start_offset and rel_time <= (start_offset + duration):
            outbag.write(topic, msg, t)
        elif rel_time > (start_offset + duration):
            break

print("Extraction completed!")