from PIL import Image, ImageDraw
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np
from rasterize import Scanline_Rasterize_Polygon, Sort_Vertices, Bounding_Box

image_resolution = 1080
res = 1080
offset = 0.5
scale_factor = image_resolution / res

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


img = Image.new('RGB', (image_resolution, image_resolution))
draw = ImageDraw.Draw(img)

seeds = []
startseeds = seeds
point_count = 100

for n in range(point_count):
    rand_x = np.random.rand() * image_resolution
    rand_y = np.random.rand() * image_resolution
    seeds.append(np.array([rand_x,rand_y]))
    center_ellipse(rand_x, rand_y,10,'white')

vor = Voronoi(seeds)

polygons = []

for region in vor.regions:
    poly = []
    if -1 in region or len(region) == 0:
        continue
    for index in region:
        if index != -1:
            poly.append(vor.vertices[index])
        else:
            print("OUTSIDE")
    
    sorted_poly = Sort_Vertices(poly)

    polygons.append(sorted_poly)

test_poly = polygons[10]


bbox = [[0,0],[image_resolution,image_resolution]]

for polygon in polygons:
    g = round(np.random.rand() * 200) + 55

    rastered_polygon, scale_factor = Scanline_Rasterize_Polygon(polygon, bbox, res, image_resolution)

    for point in rastered_polygon:
        # center_rectangle(point[0], point[1], scale_factor -2, scale_factor -2, f'rgb({g},{g},{g})')
        center_rectangle(point[0], point[1], scale_factor, scale_factor, f'rgb({g},{g},{g})')



img.save('voronoi_rasterized.png')