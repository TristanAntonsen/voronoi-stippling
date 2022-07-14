import numpy as np

def Raster_Centroid(region):
    cx = 0
    cy = 0
    for pixel in region:
        cx += pixel[0] # SHOULD BE + X
        cy += pixel[1] # SHOULD BE + Y
    cx /= len(region)
    cy /= len(region)
    return [cx, cy]




def Closest_point(search,map):
    distances = []
    sx = search[0]
    sy = search[1]
    for point in map.points:
        tx = point[0]
        ty = point[1]
        distance = np.sqrt((tx - sx)**2 + (ty-sy)**2)
        distances.append(distance)
    points = list(map.points)
    min_dist = np.min(distances)
    # print(points)
    # print(distances)
    for point, distance in zip(points,distances):
        if distance == min_dist:
            return point #closest point
