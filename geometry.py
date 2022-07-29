import numpy as np

def Line_Intersection(p1, p2, p3, p4):

    ## Line 1
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]

    ## Line 2
    x3 = p3[0]
    y3 = p3[1]
    x4 = p4[0]
    y4 = p4[1]

    ## p_x

    p_x_num = (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)
    p_x_denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    p_y_num = (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)
    p_y_denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    p_x = p_x_num / p_x_denom
    p_y = p_y_num / p_y_denom

    return (p_x, p_y)

def Calculate_Centroid(polygon):
    x = 0
    y = 0
    for vert in polygon:
        x += vert[0]
        y += vert[1]
    x /= len(polygon)
    y /= len(polygon)

    return [x,y]

def Clamp(n, range):
    return max(range[0], min(range[1], n))
    

def Create_Seed(res, threshold, image):
    while True:
        rand_x = np.random.rand() * res
        rand_y = np.random.rand() * res
        sampled_value = image.getpixel((rand_x,rand_y))
        if np.mean(sampled_value) < threshold:
            seed = np.array([rand_x,rand_y])
            return seed

def Edges(polygon):
    edges = []
    for i, point in enumerate(polygon):

        k = i + 1
        if k == len(polygon):
            k = 0
        point2 = polygon[k]

        edge = [point, point2]

        edges.append(edge)
    
    return edges

def Sort_Vertices(polygon):
    angles = []
    centroid = Calculate_Centroid(polygon)
    # vA = x vector starting at centroid
    vA = np.array([centroid[0] + 300,centroid[1],0])
    s = 5
    for vert in polygon:
        x = vert[0]
        y = vert[1]
        #vB = vector between centroid and vertex
        vB = np.array([x - centroid[0], y - centroid[1],0])
        
        numerator = np.dot(vA,vB)
        denominator = np.linalg.norm(vA) * np.linalg.norm(vB)
        theta = np.arccos(numerator / denominator)

        sign = np.cross(vA,vB) / np.linalg.norm(np.cross(vA,vB))
        theta *= sign[2]

        angles.append(theta)


    sorted_verts = np.array([x for _, x in sorted(zip(angles, polygon))])

    return sorted_verts
