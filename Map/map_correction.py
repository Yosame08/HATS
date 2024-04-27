'''
Make ID of roads continuous
'''
def renumber_ids(edge_file, waytype_file, new_edge_file, new_waytype_file):
    edge_dict = {}
    waytype_dict = {}

    with open(edge_file, 'r') as f:
        for line in f:
            data = line.strip().split()
            edge_dict[int(data[0])] = data[1:]

    with open(waytype_file, 'r') as f:
        for line in f:
            data = line.strip().split()
            waytype_dict[int(data[0])] = data[1:]

    new_id = 0
    with open(new_edge_file, 'w') as f:
        for old_id in sorted(edge_dict.keys()):
            f.write(str(new_id) + ' ' + ' '.join(edge_dict[old_id]) + '\n')
            new_id += 1

    new_id = 0
    with open(new_waytype_file, 'w') as f:
        for old_id in sorted(waytype_dict.keys()):
            f.write(str(new_id) + ' ' + ' '.join(waytype_dict[old_id]) + '\n')
            new_id += 1

# 调用函数
renumber_ids('edgeOSM_Shanghai.txt', 'wayTypeOSM_Shanghai.txt', 'new_edgeOSM_Shanghai.txt', 'new_wayTypeOSM_Shanghai.txt')

