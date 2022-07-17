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
        
        # if y < centroid[1]:
        #     theta = -theta

        sign = np.cross(vA,vB) / np.linalg.norm(np.cross(vA,vB))
        theta *= sign[2]

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

def Calculate_Angle(u,v):

    numerator = np.dot(u,v)
    denominator = np.linalg.norm(u) * np.linalg.norm(v)

    theta = np.arccos(numerator / denominator)

    return theta

##### Brute force rasterization #####
def Rasterize_Polygon(polygon, bbox, raster_res, image_res):
    scale_factor = image_res / raster_res
    points = []

    x_range = [c * scale_factor for c in range(raster_res)]
    y_range = [c * scale_factor for c in range(raster_res)]

    for x in x_range:
        for y in y_range:
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

##### Scanline rasterization #####

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


def Scanline_nodes(polygon,scan_y, image_res):

    polygon_edges = Edges(polygon)
    nodes = []
    for edge in polygon_edges:
        edge = list(edge)
        p1y = edge[0][1]
        p2y = edge[1][1]

        if p1y < scan_y and p2y >= scan_y or p1y >= scan_y and p2y < scan_y:
            p1x = edge[0][0]
            p2x = edge[1][0]

            p1 = [p1x,p1y]
            p2 = [p2x,p2y]
            p3 = [0,scan_y]
            p4 = [image_res,scan_y]
            node = Line_Intersection(p1,p2,p3,p4)

            nodes.append(node)

    if len(nodes) < 1:
        return False
    else:
        return nodes


def Raster_Scanline(nodes, scan_y, raster_res, image_res):
    scale_factor = image_res / raster_res
    x1 = min(nodes[0][0],round(nodes[1][0]))
    x2 = max(nodes[0][0],round(nodes[1][0]))

    raster_y = round(scan_y / scale_factor) * scale_factor
    raster_x = round(x1 / scale_factor) * scale_factor

    pixels = []

    while raster_x < x2:
        pixels.append([raster_x,raster_y])
        raster_x += scale_factor

    return pixels

def Scanline_Rasterize_Polygon(polygon, bbox, raster_res, image_res):

    scale_factor = image_res / raster_res
    scan_y = bbox[0][1]
    pixels = []
    while scan_y < bbox[1][1]:
        nodes = Scanline_nodes(polygon,scan_y, image_res)
        if nodes:
            pixels += Raster_Scanline(nodes, scan_y, raster_res, image_res)
        scan_y += scale_factor

    return pixels, scale_factor