# -*- coding: utf-8 -*-

# python imports
import math

# project imports
from ...ks.models import ESoundIntensity


def calculate_intensity(distance):
    return int(distance)


def calculate_distance(point_a, point_b):
    return math.sqrt(math.pow(point_a.x - point_b.x, 2) + math.pow(point_a.y - point_b.y, 2))


def int_to_intensity(intensity, max_intensities_dict):
    tmp = {val: key for (key, val) in max_intensities_dict.items()}
    max_weak = next(iter(tmp))
    if max_weak > intensity:
        return ESoundIntensity.Weak
    max_normal = next(iter(tmp))
    if max_weak <= intensity <= max_normal:
        return ESoundIntensity.Normal
    max_strong = next(iter(tmp))
    if max_strong < intensity:
        return ESoundIntensity.Strong