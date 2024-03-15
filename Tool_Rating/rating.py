'''
The rest of the code in this folder except rating.py is from previous work RNTrajRec and MTrajRec, etc.
'''
import numpy as np

from utils.evaluation_utils import cal_id_acc, cal_rn_dis_loss
from utils.shortest_path_func import SPSolver
from map import RoadNetworkMapFull

# File Path
map_root = f"../Map/"
rn = RoadNetworkMapFull(map_root, zone_range=[41.111975, -8.667057, 41.177462, -8.585305], unit_length=50)
sp_solver = SPSolver(rn, use_ray=False, use_lru=True)
file_true = '../test_output.txt'
file_pred = '../RecoveryHistory/3.13-all.txt'


# def calc_dist(lat1, lon1, lat2, lon2):
#     # 将角度转换为弧度
#     lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
#
#     # Haversine
#     dlon = lon2 - lon1
#     dlat = lat2 - lat1
#     a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
#     c = 2 * math.asin(math.sqrt(a))
#     r = 6371  # radius of earth
#
#     # 将结果转换为米
#     return c * r * 1000


def OpenTraj(filename):
    last = -1
    timestamp = 0
    with open(filename, 'r') as f:
        lines = f.readlines()
        ret = []
        for line in lines:
            if line[0] == 'F':
                assert False
            info = line.split(' ')
            if len(info) == 1 and not info[0][0] == 'F':
                if int(info[0]) - last < -2:
                    yield []
                yield ret
                ret = []
                last = int(info[0])
            else:
                # if int(info[0]) == timestamp:
                #     continue
                timestamp = int(info[0])
                ret.append(info)


def calculate_metrics(file_true, file_pred):
    true = OpenTraj(file_true)
    pred = OpenTraj(file_pred)

    recall_list = []
    precision_list = []
    f1_list = []
    accuracy_list = []
    MAE_list = []
    RMSE_list = []
    RN_MAE_list = []
    RN_RMSE_list = []
    number = 0

    id = 0
    for true_list, pred_list in zip(true, pred):
        length = len(true_list)
        # if not (length == len(pred_list) and length > 0):
        #     continue
        assert length == len(pred_list) and length > 0
        if id >= 5000:
            break
        id += 1
        number += length
        if id % 500 == 0:
            print(f"{id}\t {number} Points")
        true_path = []
        tmp_target_gps = []
        pred_path = []
        tmp_predict_gps = []
        # dists = 0
        # distssq = 0
        accu = 0
        for i in range(length):
            assert true_list[i][0] == pred_list[i][0]
            lat1, lon1, lat2, lon2 = float(true_list[i][1]), float(true_list[i][2]), float(pred_list[i][1]), float(
                pred_list[i][2])
            # dist = calc_dist(lat1, lon1, lat2, lon2)
            # dists += dist
            # MAE_all += dist
            # distssq += dist * dist
            # RMSE_all += dist * dist
            if int(true_list[i][3]) == int(pred_list[i][3]):
                accu += 1
            true_path.append(int(true_list[i][3]))
            pred_path.append(int(pred_list[i][3]))
            tmp_target_gps.append([float(true_list[i][1]), float(true_list[i][2])])
            tmp_predict_gps.append([float(pred_list[i][1]), float(pred_list[i][2])])

        mae, rmse, rn_mae, rn_rmse = cal_rn_dis_loss(sp_solver, tmp_predict_gps, pred_path,
                                                     tmp_target_gps, true_path, True)
        accuracy, recall, precision, f1 = cal_id_acc(pred_path, true_path)
        # true_set = set(true_path)
        # pred_set = set(pred_path)
        # intersection = true_set & pred_set
        # recall = len(intersection) / len(true_set)
        # precision = len(intersection) / len(pred_set)
        # f1_list.append(2 * recall * precision / (recall + precision) if precision + recall != 0 else 0)
        recall_list.append(recall)
        precision_list.append(precision)
        f1_list.append(f1)
        accuracy_list.append(accuracy)
        MAE_list.append(mae)
        RMSE_list.append(rmse)
        RN_MAE_list.append(rn_mae)
        RN_RMSE_list.append(rn_rmse)

    print(f'Average Recall: {np.array(recall_list).mean():.4f}')
    print(f'Average Precision: {np.array(precision_list).mean():.4f}')
    print(f'Average F1 Score: {np.array(f1_list).mean():.4f}')
    print(f'Average Accuracy: {np.array(accuracy_list).mean():.4f}')
    print(f'Average MAE: {np.array(MAE_list).mean():.4f}')
    print(f'Average RMSE: {np.array(RMSE_list).mean():.4f}')
    print(f'Average RoadNet MAE: {np.array(RN_MAE_list).mean():.4f}')
    print(f'Average RoadNet RMSE: {np.array(RN_RMSE_list).mean():.4f}')


calculate_metrics(file_true, file_pred)
