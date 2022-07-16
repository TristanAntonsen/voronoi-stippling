import numpy as np

def Calculate_Centroid(polygon):
    x = 0
    y = 0
    for vert in polygon:
        x += vert[0]
        y += vert[1]
    x /= len(polygon)
    y /= len(polygon)

    return [x,y]

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
        if x < centroid[0] and y < centroid[1]:
            theta = -theta
        elif x >= centroid[0] and y < centroid[1]:
            theta = -theta

        angles.append(theta)

    sorted_verts = np.array([x for _, x in sorted(zip(angles, polygon))])


    return sorted_verts


def Edge_Function(_v0, _v1, _p):
    #Assumes clockwise order
    result = (_p[0] - _v0[0]) * (_v1[1] - _v0[1]) - (_p[1] - _v0[1]) * (_v1[0] - _v0[0])
    
    if result <= 0:
        return True
    else:
        return False

def Bounding_Box(polygon):
    x_vals = []
    y_vals = []
    for vert in polygon:
        x_vals.append(vert[0])
        y_vals.append(vert[1])

    x_min = round(np.min(x_vals))
    x_max = round(np.max(x_vals)) + 1 #account for
    y_min = round(np.min(y_vals))
    y_max = round(np.max(y_vals)) + 1 #account for

    return ([x_min, x_max],[y_min,y_max])

def Rasterize_Polygon(polygon):
    
    bbox = Bounding_Box(polygon)
    points = []
    for x in range(*bbox[0]):
        for y in range(*bbox[1]):
                state = True
                vertex_count = len(polygon)

                for i in range(vertex_count):
                    v0 = i
                    v1 = i + 1
                    if v1 == vertex_count:
                        v1 = 0
                    edge_state = Edge_Function(polygon[v0],polygon[v1],[x,y])
                    state = state and edge_state
                
                ## If on positive side of all lines
                if state:
                    points.append([x,y])
                    
    return points

