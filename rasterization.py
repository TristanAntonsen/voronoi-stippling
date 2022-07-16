from re import L
from turtle import width
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import Voronoi, voronoi_plot_2d

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
    vA = np.array([0,1,0])
    for vert in polygon:
        x = vert[0]
        y = vert[1]
        vB = np.array([centroid[0] - x, centroid[1] - y,0])
        
        numerator = np.linalg.norm(np.cross(vA,vB))
        denominator = np.linalg.norm(vA) * np.linalg.norm(vB)
        theta = np.arcsin(numerator / denominator)
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


for x in range(res):
    for y in range(res):
        center_rectangle((x + 0.5) * scale_factor, (y + 0.5) * scale_factor, scale_factor - 2, scale_factor - 2, 'blue')

polygon = np.array([[15,10],[75,20],[80,40],[70,80],[50,90],[10,50]])

rasterized = Rasterize_Polygon(polygon)

for point in rasterized:
    x = point[0]
    y = point[1]

    center_rectangle((x + 0.5) * scale_factor, (y + 0.5) * scale_factor, scale_factor - 2, scale_factor - 2, 'pink')


img.save('pixels3.png')
