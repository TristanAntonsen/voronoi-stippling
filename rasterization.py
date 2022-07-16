from turtle import width
from matplotlib.pyplot import fill
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import Voronoi, voronoi_plot_2d
from rasterize import Rasterize_Polygon, Sort_Vertices, Calculate_Centroid
import time

t0 = time.time()

res = 100
image_resolution = 2160
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

        draw.line((point[0],point[1],point2[0],point2[1]),fill=color, width=width)


polygon = np.array([[15,10],[75,20],[80,40],[70,80],[50,90],[10,50]])


sorted_polygon = Sort_Vertices(polygon)

rasterized = Rasterize_Polygon(sorted_polygon)


for point in rasterized:
    x = point[0] * scale_factor
    y = point[1] * scale_factor

    # center_rectangle((x + 0.5), (y + 0.5), scale_factor / 2, scale_factor / 2, 'pink')
    center_ellipse((x + 0.5), (y + 0.5), scale_factor / 2, 'pink')
    
Outline_Poly([p * scale_factor for p in sorted_polygon],'rgb(255,255,255)', 3)

# for p in sorted_polygon:
#     center_ellipse(p[0],p[1],10,'white')
# centroid = Calculate_Centroid(polygon)
# center_ellipse(centroid[0],centroid[1],10,'white')

img.save('pixels3.png')



# seeds = []
# startseeds = seeds
# n = 0
# point_count = 50

# for n in range(point_count):
#     rand_x = np.random.rand()
#     rand_y = np.random.rand()
#     img_x = rand_x * image_resolution
#     img_y = rand_y * image_resolution
#     seed = np.array([img_x,img_y])
#     seeds.append(seed)


# vor = Voronoi(seeds)
# polygons = []

# for point in vor.points:
#     center_ellipse(point[0],point[1],10,'white')

# for region in vor.regions:
#     poly = []
#     if -1 in region or len(region) == 0:
#         continue
#     for index in region:
#         if index != -1:
#             poly.append(vor.vertices[index])
#         else:
#             print("OUTSIDE")
    
#     sorted_poly = Sort_Vertices(poly)

#     polygons.append(sorted_poly)

# # test_polygon = polygons[10]

t1 = time.time()
# for polygon in polygons:
#     rasterized = Rasterize_Polygon(polygon)
#     g = round(np.random.rand() * 255)
#     for point in rasterized:
#         x = point[0]
#         y = point[1]

#         center_rectangle((x + 0.5), (y + 0.5, f'rgb({g},{g},{g})')

#     # Outline_Poly(polygon,'rgb(255,255,255)', 3)
#     # for point in polygon:
#     #     center_ellipse(point[0],point[1],5,'white')

# # test_poly = polygons[2]
# # print(test_poly)
# # print(type(test_poly))
# # print(type(test_poly[1]))
# Outline_Poly(test_poly,'rgb(255,0,0)', 5)
# t2=time.time()
# img.save('pixels3.png')

# t3=time.time()

print(f"Time 1= {t1 - t0}s")
# print(f"Time 2= {t2 - t1}s")
# print(f"Time 3= {t3 - t2}s")
# print(f"Total time = {t3 - t0}s")