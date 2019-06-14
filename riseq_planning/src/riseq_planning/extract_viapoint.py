import numpy as np


def extract_viapoint(nmap, path):
    via_point = []

    for point in path:
        if nmap[point[0]+1][point[0] + 1] == 1 or nmap[point[0]+1][point[0] - 1] == 1 \
                or nmap[point[0]-1][point[0] + 1] == 1 or nmap[point[0]-1][point[0]-1] == 1:
            via_point.append(point)

    return via_point
