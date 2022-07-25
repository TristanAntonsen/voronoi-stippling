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

def Raster_Centroid(region):
    cx = 0
    cy = 0
    for pixel in region:
        cx += pixel[0] # SHOULD BE + X
        cy += pixel[1] # SHOULD BE + Y
    cx /= len(region)
    cy /= len(region)
    return [cx, cy]

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

def Raster_BBox(polygon, image_res):
    x_vals = []
    y_vals = []
    for vert in polygon:
        x_vals.append(vert[0])
        y_vals.append(vert[1])

    x_min = round(np.min(x_vals))
    x_max = round(np.max(x_vals)) #account for
    y_min = round(np.min(y_vals))
    y_max = round(np.max(y_vals)) #account for

    if x_min < 0:
        x_min = 0
    if y_min < 0:
        y_min = 0
    if x_max >= image_res:
        x_max = image_res - 1
    if y_max >= image_res:
        y_max = image_res - 1
        
    return ([x_min, y_min],[x_max,y_max])


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

def Clamp(n, range):
    return max(range[0], min(range[1], n))

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

            if p1x < 0:
                p1x = 0
            if p1x >= image_res:
                p1x = image_res - 1

            p1 = [p1x,p1y]
            p2 = [p2x,p2y]
            p3 = [0,scan_y]
            p4 = [image_res,scan_y]
            node = Line_Intersection(p1,p2,p3,p4)
            node = [Clamp(node[0],[0,image_res]),Clamp(node[1],[0,image_res])]
            nodes.append(node)

    if len(nodes) < 1:
        return False
    else:
        return nodes


def Raster_Scanline(nodes, scan_y):

    x1 = min(nodes[0][0],nodes[1][0])
    x2 = max(nodes[0][0],nodes[1][0])

    raster_y = scan_y
    raster_x = x1

    pixels = []

    while raster_x < x2:
        pixels.append([int(raster_x),int(raster_y)])
        raster_x += 1

    return pixels

def Scanline_Rasterize_Polygon(polygon, bbox, image_res):

    scan_y = bbox[0][1]
    pixels = []
    # y_inc = (bbox[1][1] - bbox[0][1]) / 2
    while scan_y < bbox[1][1]:
        nodes = Scanline_nodes(polygon,scan_y, image_res)
        if nodes:
            pixels += Raster_Scanline(nodes, scan_y)
        scan_y += 1
        # scan_y += y_inc

    return pixels

def Weighted_Raster_Centroid(pixels, image_array):
    cx = 0
    cy = 0
    total_weight = 0
    pixel_count = len(pixels)
    increment_size = round(pixel_count / 500)
    if increment_size < 1:
        increment_size = 1
    i = 0
    while i < pixel_count:
        pixel = pixels[i]
        sample_x = pixel[1]
        sample_y = pixel[0]
        # sampled_value = image.getpixel((sample_x,sample_y))
        sampled_value = image_array[sample_x][sample_y]

        # weight = 1 - np.mean(sampled_value) / 255
        weight = 1 - sampled_value


        total_weight += weight
        
        cx += pixel[0] * weight
        cy += pixel[1] * weight

        i += increment_size
        # i += 1
    
    if total_weight == 0:
        total_weight = pixel_count

    cx /= total_weight
    cy /= total_weight
    
    return [cx, cy]