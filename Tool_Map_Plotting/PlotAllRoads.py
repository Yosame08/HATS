import folium
from folium.plugins import MarkerCluster
from file import *

mapfile = '../Map/edgeOSM_Shanghai.txt'
generateM = 'shanghai_roads_full.html'

specify = False
indexes = [0]


def TrackToLayer(tracks: list[Anchors]) -> list[dict]:
    trajectories = []
    for track in tracks:
        trajectories.append({
            'name': 'Trajectory ' + str(track.id),
            'points': track.anchor
        })


# Set up the map centered on Shanghai
print("STEP 0: Creating Map")
map = folium.Map(location=[31.26, 121.55], zoom_start=15)
roadnet = RoadNet()

if len(mapfile) > 0:
    print("STEP 1: Reading Roadnet")
    roadnet = mapfile_DSPJ(mapfile)
    print(roadnet.road[0].segment)

for road in roadnet.road:
    line = folium.PolyLine(road.segment, color='#66ccff', weight=5)
    line.add_child(folium.Popup(f"Road ID: {road.id}"))
    line.add_to(map)

print("STEP 3: Saving")
map.save(generateM)
