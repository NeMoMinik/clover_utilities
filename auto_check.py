import requests as rq

topics = ['http://127.0.0.1:8080/snapshot?topic=/aruco_map/debug',
'http://127.0.0.1:8080/snapshot?topic=/aruco_detect/debug',
'http://127.0.0.1:8080/snapshot?topic=/aruco_map/image'
         ]

for i in topics:
    try:
        rq.get(i).status_code
    except:
        print('\033[93mTopic', i[i[:-6].rfind('/'):], 'is offline.')
    else:
        print('\033[92mTopic', i[i[:-6].rfind('/'):], 'is online.')

with open('/home/clover/catkin_ws/src/clover/clover/launch/aruco.launch') as f:
    lines = f.readlines()
    length = lines[5]
    maps = lines[6]

length = float(length[32:length.rfind('"')])
maps = maps[29:maps.rfind('"')]

with open('/home/clover/catkin_ws/src/clover/aruco_pose/map/{}'.format(maps)) as f:
    lines = f.readlines()
    length2 = lines[1]
    length2 = float(length2[2:length2.find('	', 2)])

print(['\033[93mMarker length in aruco.launch and aruco map must be equal: {} vs {}.'.format(length2, length),
       '\033[92mMarker length is equal.'][length2 == length])
