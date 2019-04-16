'''
These are just storing some basic data structure and algorithms.
After I practice leetcode, I'll add more to here.
'''
import numpy as np

# find x, or find the last element that is smaller than x
def BinarySearch(array,x): 
    l = 0
    r = len(array)-1
    while l < r:
        mid = l+(r-l)/2
        if array[mid] < x:
            l = mid + 1
        elif array[mid] > x:
            r = mid - 1
        else:
            return (mid, array[mid])
    return (r, array[r])

# find the first element that is not smaller than x
def lower_bound(array,x): 
    if array[-1]<x:
        return None
    l = 0
    r = len(array)-1
    while l < r:
        mid = l+(r-l)/2
        if array[mid] < x:
            l = mid + 1
        else:
            r = mid
    return (r, array[r])
    

if __name__=="__main__":
    d=[1,2,2,2,2,360,900,2000]
    idx, val = lower_bound(d, 2)
    print(idx, val)
    idx, val = lower_bound(d, 2+0.001)
    print(idx, val)
    idx, val = BinarySearch(d, 2)
    print(idx, val)
    idx, val = BinarySearch(d, 2+0.001)
    print(idx, val)