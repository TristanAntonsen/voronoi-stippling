from turtle import width
import numpy as np
from PIL import Image, ImageDraw
from rasterize import Rasterize_Polygon, Sort_Vertices, Calculate_Centroid
import time

t0 = time.time()

res_init = 100
image_resolution = 2160
offset = 0.5
scale_factor_init = image_resolution / res_init

img = Image.new('RGB', (image_resolution, image_resolution))
draw = ImageDraw.Draw(img)

def center_rectangle(x,y,l,w,c):
    l = l/2
    w = w/2
    draw.rectangle([x - w, y - l, x + w, y + l],fill=c)

def center_ellipse(x,y,r,c):

    draw.ellipse([x - r, y - r, x + r, y + r],fill=c)


def Outline_Poly(polygon,color,width):
    for i, point in enumerate(polygon):

        k = i + 1
        if k == len(polygon):
            k = 0
        point2 = polygon[k]

        draw.line((point[0],point[1],point2[0],point2[1]),fill=color, width=width)


polygon = np.array([[15,10],[75,20],[80,40],[70,80],[50,90],[10,50]])
polygon = [p * scale_factor_init for p in polygon]

sorted_polygon = Sort_Vertices(polygon)



# for point in rasterized:
#     x = point[0] * scale_factor
#     y = point[1] * scale_factor

    # center_rectangle((x + 0.5), (y + 0.5), scale_factor / 2, scale_factor / 2, 'pink')
    # center_ellipse((x + 0.5), (y + 0.5), scale_factor / 2, 'pink')
    
# Outline_Poly(sorted_polygon,'rgb(255,255,255)', 3)

# for p in sorted_polygon:
#     center_ellipse(p[0],p[1],10,'white')



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


def Scanline_nodes(polygon,scan_y):

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
            p4 = [image_resolution,scan_y]
            node = Line_Intersection(p1,p2,p3,p4)

            nodes.append(node)

    if len(nodes) < 1:
        return False
    else:
        return nodes

def Visualize_Scanline(nodes, scan_y):
    x1 = min(round(nodes[0][0]),round(nodes[1][0]))
    x2 = max(round(nodes[0][0]),round(nodes[1][0]))
    x = x1
    draw.line((x1,scan_y,x2,scan_y),fill='white',width=10)


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

def Rasterize_Polygon(polygon, bbox, raster_res, image_res):

    scale_factor = image_res / raster_res
    scan_y = bbox[0][1]
    pixels = []
    while scan_y < bbox[1][1]:
        nodes = Scanline_nodes(polygon,scan_y)
        if nodes:
            pixels += Raster_Scanline(nodes, scan_y, raster_res, image_resolution)
        scan_y += scale_factor
    return pixels, scale_factor

bbox = [[0,0],[image_resolution,image_resolution]]
raster_res = 100

rastered_polygon, scale_factor = Rasterize_Polygon(sorted_polygon, bbox, raster_res, image_resolution)

for point in rastered_polygon:
    center_rectangle(point[0], point[1], scale_factor -2, scale_factor -2, "white")


img.save('raster2.png')
