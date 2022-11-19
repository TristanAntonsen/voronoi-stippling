from PIL import Image, ImageDraw, ImageFilter
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np
from geometry import Sort_Vertices
from rasterize import Scanline_Rasterize_Polygon, Raster_BBox, Weighted_Raster_Centroid
import time
import cProfile
from stippling import Stippler


if __name__ == "__main__":
    image_path = 'sampling/circle_1080.jpg'
    image_resolution = 1080
    polygon = np.array([[10,10],[1000,10],[1000,1000],[10,1000]])
    t0 = time.time()
    bbox = Raster_BBox(polygon, image_resolution)
    rasterized = Scanline_Rasterize_Polygon(polygon,bbox,image_resolution)
    weight_array = np.zeros([image_resolution,image_resolution])
    img = Image.new('RGB', (image_resolution, image_resolution))
    image_array = np.array(img)
    for x in range(image_resolution):
        for y in range(image_resolution):
            pixel = image_array[x,y]
            weight_array[x,y] = np.mean(pixel) / 255
    t1 = time.time()
    cProfile.run("Weighted_Raster_Centroid(rasterized, weight_array)","output.dat")
    t2 = time.time()

    print(f"Elapsed: {round(t2 - t0,2)}")
    print(f"Init: {round(t1 - t0,2)}")
    import pstats
    from pstats import SortKey

    with open("output_time.txt", "w") as f:
        p = pstats.Stats("output.dat",stream=f)
        p.sort_stats("time").print_stats()
    
    with open("output_calls.txt", "w") as f:
        p = pstats.Stats("output.dat",stream=f)
        p.sort_stats("calls").print_stats()

