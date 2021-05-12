import os


def get(oxt_dir):
    ts_dir = os.path.join(oxt_dir, 'timestamps.txt')

    lines = {}
    with open(ts_dir, 'r') as f:
        lines = f.readlines()
        data = {idx: line.split('\n')[0] for idx, line in enumerate(lines)}

    return data