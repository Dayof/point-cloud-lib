import os
from tqdm import tqdm

BASE_PATH = os.getenv('BASE_PATH', False)
VELODYNE_PATH = os.path.join(BASE_PATH, 'velodyne_points')
OXTS_PATH = os.path.join(BASE_PATH, 'oxts')

class KITTIStruct:
    def __init__(self, n_vtx):
        self.vtxs = {idx: {'timestamp':'', 'lat': '', 'lon': '', 'ref': ''} for idx in range(n_vtx)}

    def set_timestamps(self, timestamps):
        for (k, v), timestamp in zip(self.vtxs.items(), timestamps):
            v['timestamp'] = timestamp

    def set_metadata(self, metadata):
        for idx, info in self.vtxs.items():
            info_metadata = metadata[info['timestamp']]
            info['ref'] = info_metadata['ref']
            info['lat'], info['lon'] = info_metadata['lat'], info_metadata['lon']

    def __str__(self):
        str_rslt = ''
        for k, v in self.vtxs.items():
            str_rslt += f'Vertex: {k}\nTimestamp: {v["timestamp"]}\n'\
                        f'Lat, Lon: {v["lat"]}, {v["lon"]}\nFile ref: {v["ref"]}\n\n'
        return str_rslt
        
def get_num_vtxs():
    vtx_file = os.path.join(BASE_PATH, 'vtx.txt')
    nr = 0
    with open(vtx_file, 'r') as f:
        nr = f.readline()
    print(nr, 'vertexes found.')
    return int(nr)

# time format: HH:MM:SS.s
def format_timestamp(line):
    new_line = line.strip('\n').split('.')
    timestamp = new_line[0]
    timestamp += '.' + new_line[1][0]
    return timestamp

def get_velodyne_timestamps():
    time_file = os.path.join(VELODYNE_PATH, 'timestamps.txt')
    lines = []
    with open(time_file, 'r') as f:
        lines = [format_timestamp(l) for l in f.readlines() if l.strip(' \n') != '']
    return lines

def get_oxts_lat_long_by_timestamp(timestamps):

    def filter_by_velodyne_timestamps(lines):
        new_lines, flags = {}, []
        for idx, line in lines.items():
            if line not in flags and line in timestamps:
                flags.append(line)
                new_lines[idx] = line
        return new_lines

    def get_lat_lon(lines):
        all_lat_lon = {}

        def get_file_number(f):
            return int(f.split('.')[0])

        data_path = os.path.join(OXTS_PATH, 'data')
        for f in os.listdir(data_path):
            nr = get_file_number(f)
            if nr in lines:
                file_path = os.path.join(data_path, f)
                lat, lon = '', ''
                with open(file_path, 'r') as df:
                    cur_line = df.readline().split(' ')
                    lat, lon = cur_line[0], cur_line[1]
                all_lat_lon[lines[nr]] = {'lat': lat, 'lon': lon, 'ref': f}
        return all_lat_lon

    time_file = os.path.join(OXTS_PATH, 'timestamps.txt')
    lines = []
    with open(time_file, 'r') as f:
        lines = {idx: format_timestamp(l) for idx, l in enumerate(f.readlines(), 1) if l.strip(' \n') != ''}
    filtered_timestamps = filter_by_velodyne_timestamps(lines)
    return get_lat_lon(filtered_timestamps)

def start():
    if not BASE_PATH:
        print('Please set the BASE_PATH first.')
        quit()

    n_vtx = get_num_vtxs()
    kitti_struct = KITTIStruct(n_vtx)
    timestamps = get_velodyne_timestamps()
    kitti_struct.set_timestamps(timestamps)
    kitti_struct.set_metadata(get_oxts_lat_long_by_timestamp(timestamps))
    print(kitti_struct)

if __name__ == "__main__":
    start()