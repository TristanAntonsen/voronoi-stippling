from turtle import width
import numpy as np
from PIL import Image, ImageDraw
from rasterize import Rasterize_Polygon, Scanline_Rasterize_Polygon, Sort_Vertices, Calculate_Centroid, Edges, Raster_Scanline, Scanline_nodes
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


def Visualize_Scanline(nodes, scan_y):
    x1 = min(round(nodes[0][0]),round(nodes[1][0]))
    x2 = max(round(nodes[0][0]),round(nodes[1][0]))
    x = x1
    draw.line((x1,scan_y,x2,scan_y),fill='white',width=10)


bbox = [[0,0],[image_resolution,image_resolution]]
raster_res = 100

rastered_polygon, scale_factor = Scanline_Rasterize_Polygon(sorted_polygon, bbox, raster_res, image_resolution)

for point in rastered_polygon:
    center_rectangle(point[0], point[1], scale_factor -2, scale_factor -2, "white")


img.save('raster2.png')
