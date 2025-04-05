'''
Saketh Ayyagari
Utilities that can be used throught any node
'''

'''
Remaps a value within another range [c,d] given a value 
from one range [a, b]
'''
def remap_range(value, a, b, 
                c, d):
    scale_factor = (d - c) / (b - a)
    return clamp((scale_factor * (value - a)) + c, c, d)
'''
Clamps a value within a specific range
Lower values become the minimum while higher values become the maximum
'''
def clamp(value, low, high):
    if value > high:
        return high
    elif value < low:
        return low
    return value