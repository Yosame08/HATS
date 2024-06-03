import itertools
from curses.ascii import isdigit

Roadnet_Road_SegCount_Index = 3
Roadnet_Road_Seg_StartFrom = 4
# Roadnet_Road_SegCount_Index = 5
# Roadnet_Road_Seg_StartFrom = 6
Anchors_Ignore_Time = -0x7FFFFFFF


class Anchors:
    def __init__(self, id=-1):
        self.cnt = 0
        self.anchor = []
        self.id = id

    def push(self, x, y):
        self.cnt += 1
        self.anchor.append((x, y))


class RoadNet:
    class Road:
        def __init__(self, id=-1, fr=-1, to=-1, level="None"):
            self.id = id
            self.fr = fr
            self.to = to
            self.level = level
            self.cnt = 0
            self.segment = []

    def __init__(self):
        self.road: list[Road] = []

    def push(self, id, fr, to, level, anchors):
        r = RoadNet.Road(id, fr, to, level)
        for anchor in anchors:
            r.cnt += 1
            r.segment.append(anchor)
        self.road.append(r)


def maplines_DSPJ(lines):
    roadnet = RoadNet()  # 初始化路网类
    # cnt = int(lines[0])
    # for i in range(1,cnt+1):
    for i in range(len(lines)):
        fields = lines[i].strip().split()
        anchors = []
        for j in range(Roadnet_Road_Seg_StartFrom,
                       Roadnet_Road_Seg_StartFrom + int(fields[Roadnet_Road_SegCount_Index]) * 2, 2):
            anchors.append((float(fields[j]), float(fields[j + 1])))
        roadnet.push(int(fields[0]), int(fields[1]), int(fields[2]), fields[3], anchors)
    return roadnet


def anchorlines_DSPJ(lines, setid=-1):
    anchors = Anchors(setid)
    lst_anchor_tim = -10000
    for i in range(0, len(lines) - 1):
        fields = lines[i].strip().split()
        if (int(fields[0]) - lst_anchor_tim > Anchors_Ignore_Time):
            anchors.push(float(fields[1]), float(fields[2]))
            lst_anchor_tim = int(fields[0])
    return anchors


def mapfile_DSPJ(filename) -> RoadNet:
    with open(filename, 'r') as file:
        lines = file.readlines()
        # index = int(lines[0]) # 路网的道路条数
        # roadnet = maplines_DSPJ(lines[:index+1])
        roadnet = maplines_DSPJ(lines)
        return roadnet
    return None


def answer_roadid(line, roadnet: RoadNet, ID) -> list[dict]:
    all_segments = []
    if not isdigit(line[0]):
        return all_segments
    segments = line.strip().split()
    delta = 255 // len(segments)
    cnt, last = 0, -1
    for j in range(0, len(segments)):
        now = int(segments[j])
        if last == now: continue
        last = now
        cnt += 1
        new_seg = {}
        new_seg['id'] = now
        new_seg['points'] = []
        for k in roadnet.road[now].segment:
            new_seg['points'].append([k[0], k[1]])
        red, blue = 255 - j * delta, j * delta
        new_seg['color'] = f'#00{red:02x}ff'
        all_segments.append(new_seg)
    return all_segments


def anchorLines_TimePxPy(lines, setid=-1) -> Anchors:
    anchors = Anchors(setid)
    lst_anchor_tim = -0x7FFFFFFF
    if lines[0][0] == 'F':
        return anchors
    for i in range(0, len(lines)):
        fields = lines[i].strip().split()
        if (int(fields[0]) - lst_anchor_tim > Anchors_Ignore_Time):
            anchors.push(float(fields[1]), float(fields[2]))
            lst_anchor_tim = int(fields[0])
    return anchors


def anchorFile(filename) -> list[Anchors]:
    with open(filename, 'r') as file:
        lines = list(itertools.islice(file, 2000000))[1:]
        while len(lines[-1]) > 10:  # 如果最后一行长度大于10
            lines.append(next(file))  # 继续读取下一行

        tracks = []
        split = [-1]
        for i in range(0, len(lines)):
            stripped = lines[i].strip()
            if len(stripped.split(' ')) == 1 and not stripped[0] == 'F':
                split.append(i)
        for i in range(1, len(split)):
            begin = split[i - 1] + 1
            end = split[i]
            tracks.append(anchorLines_TimePxPy(lines[begin:end], i))
        return tracks
    return None
