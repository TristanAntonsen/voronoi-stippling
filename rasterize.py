import numpy as np
from geometry import Edges, Line_Intersection, Clamp

def Raster_BBox(polygon, image_res):
    x_vals = []
    y_vals = []
    for vert in polygon:
        x_vals.append(vert[0])
        y_vals.append(vert[1])

    x_min = round(np.min(x_vals))
    x_max = round(np.max(x_vals))
    y_min = round(np.min(y_vals))
    y_max = round(np.max(y_vals))

    if x_min < 0:
        x_min = 0
    if y_min < 0:
        y_min = 0
    if x_max >= image_res:
        x_max = image_res - 1
    if y_max >= image_res:
        y_max = image_res - 1
        
    return ([x_min, y_min],[x_max,y_max])

##### Scanline rasterization #####

def Scanline_nodes(polygon,scan_y, image_res):

    polygon_edges = Edges(polygon)
    nodes = []
    for edge in polygon_edges:
        edge = list(edge)
        p1y = edge[0][1]
        p2y = edge[1][1]

        ## If line crosses scanline
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

    x1 = (min(nodes[0][0],nodes[1][0]))
    x2 = (max(nodes[0][0],nodes[1][0]))

    raster_y = scan_y
    raster_x = x1 + 0.5 # pixel offset, prevent drift

    pixels = []

    while raster_x < x2:
        pixels.append([raster_x,raster_y])
        raster_x += 1

    return pixels

def Scanline_Rasterize_Polygon(polygon, bbox, image_res):

    scan_y = bbox[0][1]
    pixels = []
    while scan_y < bbox[1][1]:
        nodes = Scanline_nodes(polygon,scan_y, image_res)
        if nodes:
            pixels += Raster_Scanline(nodes, scan_y)
        scan_y += 1

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
        sample_x = int(pixel[1])
        sample_y = int(np.floor(pixel[0]))
        sampled_value = image_array[sample_x][sample_y]

        weight = 1 - sampled_value

        total_weight += weight
        
        cx += pixel[0] * weight
        cy += pixel[1] * weight

        # i += increment_size
        i += 1
    

    if total_weight == 0:
        total_weight = pixel_count

    if pixel_count == 0 or cx == 0 or cy == 0:
        return False

    cx /= total_weight
    cy /= total_weight


    return [cx, cy]

    