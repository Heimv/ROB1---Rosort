import threading
import numpy as np


class Distances:
    def __init__(self, safe_dist, critical_dist):
        self._mutex = threading.Lock()
        self._ranges = {}
        self._dist = {
            'front': 3.5,
            'left': 3.5,
            'frontleft': 3.5,
            'right': 3.5,
            'frontright': 3.5,
            'back': 3.5,
            'backleft': 3.5,
            'backright': 3.5
        }
        self.safe_dist = safe_dist
        self.critical_dist = critical_dist

    @property
    def distances(self):
        self._mutex.acquire()
        dist = self._dist
        self._mutex.release()
        return dist

    def is_critical(self, regions):
        if type(regions) == str:
            return self._dist[regions] < self.critical_dist
        criticals = []
        for region in regions:
            criticals.append(self._dist[region] < self.critical_dist)
        return True in criticals

    def average(self, regions, asum = False):
        if type(regions) == str:
            return np.average(self._ranges[regions])
        ranges = []
        for region in regions:
            ranges.append(self._ranges[region])
        if asum:
            return np.average(ranges)
        else:
            averages = []
            for region in ranges:
                averages.append(np.average(region))
            return averages

    def ranges(self, ranges):
        self._mutex.acquire()
        self._ranges = {
            'front': ranges[0],
            'frontleft': ranges[1],
            'left': ranges[2],
            'backleft': ranges[3],
            'back': ranges[4],
            'backright': ranges[5],
            'right': ranges[6],
            'frontright': ranges[7]
        }
        self._dist = {
            'front': min(self._ranges['front']),
            'frontleft': min(self._ranges['frontleft']),
            'left': min(self._ranges['left']),
            'backleft': min(self._ranges['backleft']),
            'back': min(self._ranges['back']),
            'backright': min(self._ranges['backright']),
            'right': min(self._ranges['right']),
            'frontright': min(self._ranges['frontright'])
        }
        self._mutex.release()
