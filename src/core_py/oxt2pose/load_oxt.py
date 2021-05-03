import os

import load_timestamps
import utils


def get(base_dir):
    oxt_path = os.path.join(base_dir, 'oxts')
    oxt_data_path = os.path.join(oxt_path, 'data')
    ts = load_timestamps.get(oxt_path)
    print('[OXT]\n', ts[0])
    oxts  = {}

    for i in range(len(ts)):
        if len(ts[i]) != 0:
            filename = utils.idx2filename(i, 'txt')
            filepath = os.path.join(oxt_data_path, filename)
            oxts[i] = utils.txtlines2lists(filepath)
        else:
            oxts[i] = []

    return oxts
