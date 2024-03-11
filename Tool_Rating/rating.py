from curses.ascii import isdigit
import math


def calc_dist(lat1, lon1, lat2, lon2):
    # 将角度转换为弧度
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # radius of earth

    # 将结果转换为米
    return c * r * 1000


def OpenTraj(filename):
    last = -1
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
    number = 0
    MAE_all = 0
    RMSE_all = 0

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
        pred_path = []
        dists = 0
        distssq = 0
        accu = 0
        for i in range(length):
            assert true_list[i][0] == pred_list[i][0]
            lat1, lon1, lat2, lon2 = float(true_list[i][1]), float(true_list[i][2]), float(pred_list[i][1]), float(
                pred_list[i][2])
            dist = calc_dist(lat1, lon1, lat2, lon2)
            dists += dist
            distssq += dist * dist
            MAE_all += dist
            RMSE_all += distssq
            if int(true_list[i][3]) == int(pred_list[i][3]):
                accu += 1
            true_path.append(int(true_list[i][3]))
            pred_path.append(int(pred_list[i][3]))
        # 将true_path和pred_path转换为集合，去除重复元素
        true_set = set(true_path)
        pred_set = set(pred_path)

        # 求交集
        intersection = true_set & pred_set

        # 计算recall, precision和F1 Score
        recall = len(intersection) / len(true_set)
        precision = len(intersection) / len(pred_set)

        recall_list.append(recall)
        precision_list.append(precision)
        f1_list.append(2 * recall * precision / (recall + precision) if precision + recall != 0 else 0)
        accuracy_list.append(accu / length)

        MAE = dists/length
        RMSE = math.sqrt(distssq / length)
        assert MAE > 0 and RMSE > 0
        MAE_list.append(MAE)
        RMSE_list.append(RMSE)

    avg_recall = sum(recall_list) / len(recall_list)
    avg_precision = sum(precision_list) / len(precision_list)
    avg_f1 = sum(f1_list) / len(f1_list)
    avg_accuracy = sum(accuracy_list) / len(accuracy_list)
    avg_MAE = sum(MAE_list) / len(MAE_list)
    avg_RMSE = sum(RMSE_list) / len(RMSE_list)

    print(f'Average Recall: {avg_recall:.4f}')
    print(f'Average Precision: {avg_precision:.4f}')
    print(f'Average F1 Score: {avg_f1:.4f}')
    print(f'Average Accuracy: {avg_accuracy:.4f}')
    print(f'Average MAE: {avg_MAE:.4f}')
    print(f'Average RMSE: {avg_RMSE:.4f}')
    print(f'Average MAE(all): {MAE_all/number:.4f}')
    print(f'Average RMSE(all): {math.sqrt(RMSE_all/number):.4f}')


# File Path
file_true = '../test_output.txt'
file_pred = '../RecoveryHistory/3.10NewTurn(5000).txt'

calculate_metrics(file_true, file_pred)
