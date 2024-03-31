import folium
from folium.plugins import MarkerCluster
from file import *

file_map = 'edgeOSM_Porto.txt'
file_in_sampled = '../test_sampled.txt'  # matched trace file name
file_in_pred = '../RecoveryHistory/3.30-3.txt'
file_in_true = '../test_output.txt'
file_out = '../RecoveryHistory/3.30-3Full.txt'  # continuous roads file name
file_generateM = 'Porto_Matched.html'

specify = False
track_from, track_to, track_step = 1, 30, 6  # define show how many traces on the map
indexes = [0]
plot_indexes = []
if specify:
    plot_indexes = indexes
else:
    plot_indexes = list(range(track_from, track_to, track_step))


def TrackToLayer(tracks: list[Anchors]) -> list[dict]:
    trajectories = []
    for track in tracks:
        trajectories.append({
            'name': 'Trajectory ' + str(track.id),
            'points': track.anchor
        })


# Set up the map centered on Shanghai
print("STEP 0: Creating Map")
# map = folium.Map(location=[31.26, 121.55], zoom_start=15)
map = folium.Map(location=[41.15, -8.6], zoom_start=15)
roadnet = RoadNet()

if len(file_map) > 0:
    print("STEP 1: Reading Roadnet")
    roadnet = mapfile_DSPJ(file_map)
    print(roadnet.road[0].segment)


def add_marker(filename, colorR, colorB, type):
    anchors = anchorFile(filename)
    for i in plot_indexes:
        if i >= len(anchors):
            print("\033[93m" + "! WARNING: Index %d out of the bound of traces. Ignore." % i + "\033[0m")
            break
        anchor = anchors[i]
        print("* INFO: Track %d has %d points" % (i, len(anchor.anchor)))
        if len(anchor.anchor) == 0:
            continue
        delta = 255 // len(anchor.anchor)
        for j, p in enumerate(anchor.anchor):
            red, blue = 255 - j * delta, j * delta
            color = f'#{colorR}{red:02x}{colorB}'
            marker = type(p, radius=8, color=color, fill_opacity=0.5)
            marker.add_child(folium.Popup(f"ID: {anchor.id}\ncnt: {j}"))  # Add a popup to the marker
            marker.add_to(map)


if len(file_in_true) > 0:
    print("STEP 2: Reading Tracks")
    add_marker(file_in_sampled, "00", "00", folium.Marker)
    add_marker(file_in_pred, "00", "ff", folium.CircleMarker)
    add_marker(file_in_true, "ff", "00", folium.CircleMarker)

if len(file_out) > 0:
    print("STEP 3: Processing Outputs")
    with open(file_out, 'r') as file:
        answer_file = list(itertools.islice(file, 50000))
        result = answer_file[1:]  # 忽略答案中的轨迹条数
        for i in plot_indexes:
            road_segments = answer_roadid(result[i], roadnet, i)
            print(road_segments)
            for segment in road_segments:
                line = folium.PolyLine(segment['points'], color=segment['color'], weight=5, fill_opacity=0.5)
                line.add_child(folium.Popup(f"Road ID: {segment['id']}"))
                line.add_to(map)

print("STEP 4: Saving")
map.save(file_generateM)

# # Define your road segments
# road_segments = [
#     {
#         'id': 1,
#         'start_intersection': 5152,
#         'end_intersection': 0,
#         'class_name': 'residential',
#         'class_number': 1,
#         'points': [
#             [31.2610903, 121.5570880],
#             [31.2611567, 121.5570230],
#             [31.2619722, 121.5564754],
#             [31.2633190, 121.5555711]
#         ],
#         'color': 'red'
#     }
# ]

# for trajectory in trajectories:
#     # Create a feature group for the trajectory
#     group = folium.FeatureGroup(name=trajectory['name'])
#
#     # Add a polyline to the group representing the trajectory
#     folium.PolyLine(trajectory['points'], color=trajectory['color']).add_to(group)
#
#     # Add the group to the map
#     group.add_to(map)
