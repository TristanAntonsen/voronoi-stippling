from turtle import width
from matplotlib.pyplot import fill
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import Voronoi, voronoi_plot_2d
from rasterize import Rasterize_Polygon, Sort_Vertices, Calculate_Centroid
import time
res = 100
image_resolution = 1080
offset = 0.5
scale_factor = image_resolution / res

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
        # point = [p * scale_factor for p in point]
        # point2 = [p * scale_factor for p in point2]

        draw.line((point[0],point[1],point2[0],point2[1]),fill=color, width=width)

for x in range(res):
    for y in range(res):
        center_rectangle((x + 0.5) * scale_factor, (y + 0.5) * scale_factor, scale_factor, scale_factor, 'blue')

# polygon = np.array([[15,10],[75,20],[80,40],[70,80],[50,90],[10,50]])

# rasterized = Rasterize_Polygon(polygon)


# for point in rasterized:
#     x = point[0]
#     y = point[1]

#     center_rectangle((x + 0.5) * scale_factor, (y + 0.5) * scale_factor, scale_factor, scale_factor, 'pink')

# sorted_polygon = Sort_Vertices([p*scale_factor for p in polygon])
# [print(v,s) for v,s in zip(polygon,sorted_polygon)]

# Outline_Poly(sorted_polygon,'rgb(255,255,255)', 3)

# for p in sorted_polygon:
#     center_ellipse(p[0],p[1],10,'white')
# centroid = Calculate_Centroid(polygon)

# center_ellipse(centroid[0] * scale_factor,centroid[1] * scale_factor,10,'white')

# img.save('pixels3.png')



seeds = []
startseeds = seeds
n = 0
point_count = 50

for n in range(point_count):
    rand_x = np.random.rand()
    rand_y = np.random.rand()
    img_x = rand_x * image_resolution
    img_y = rand_y * image_resolution
    seed = np.array([img_x,img_y])
    seeds.append(seed)


vor = Voronoi(seeds)
polygons = []

for point in vor.points:
    center_ellipse(point[0],point[1],10,'white')

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

# test_polygon = polygons[10]

for polygon in polygons:
    # rasterized = Rasterize_Polygon(polygon)
    g = round(np.random.rand() * 255)
    # for point in rasterized:
    #     x = point[0]
    #     y = point[1]

    #     center_rectangle((x + 0.5) * scale_factor, (y + 0.5) * scale_factor, scale_factor, scale_factor, f'rgb({g},{g},{g})')

    Outline_Poly(polygon,'rgb(255,255,255)', 3)
    for point in polygon:
        center_ellipse(point[0],point[1],5,'white')

test_poly = polygons[2]
print(test_poly)
print(type(test_poly))
print(type(test_poly[1]))
Outline_Poly(test_poly,'rgb(255,0,0)', 5)

img.save('pixels3.png')