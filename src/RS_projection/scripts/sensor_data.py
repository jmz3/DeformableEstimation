import rosbag

def GetData(msg):
    point_list = []
    print(len(msg.poses))
    for i in range(0, len(msg.poses)):
        point_list.append([msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].position.z]) 
    
    return point_list


if __name__ == "__main__":
    bag  = rosbag.Bag('NDI.bag', 'r')
    for topic, msg, t in bag.read_messages(topics = '/NDI/measured_cp_array'):
        if t.secs == 1677095866 and t.nsecs == 538592790:
            point_list = GetData(msg)
            print(point_list)

    bag.close
# 1677095866478416186
# 1677095866498345241
# 1677095866518449400
# 1677095866538592790