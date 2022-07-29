from PIL import Image, ImageDraw, ImageFilter
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np
from geometry import Sort_Vertices
from rasterize import Scanline_Rasterize_Polygon, Raster_BBox, Weighted_Raster_Centroid
import time

class Stippler:
    def __init__(self, image_path, **kwargs):
        
        self.weight_image = Image.open(image_path)
        self.image_resolution = self.weight_image.size[0]
        self._Pixel_Array(kwargs['power'])

    def Create_Seeds(self, point_count, threshold):
        res = self.image_resolution
        seeds = [
            np.array([-res, -res]),
            np.array([2 * res, -res]),
            np.array([2 * res, 2 * res]),
            np.array([-res, 2 * res])
        ]
        n = 0
        while n < point_count:
            seed = self._Create_Seed(self.image_resolution, threshold, self.weight_image)
            seeds.append(seed)
            n += 1
        self.Save_Result('sequence/seeds.png',seeds)
        self.seeds = seeds

    def _Pixel_Array(self, power):
            
        image_array = np.array(self.weight_image)
        weight_array = np.zeros([self.image_resolution,self.image_resolution])
        for x in range(self.image_resolution):
            for y in range(self.image_resolution):
                pixel = image_array[x,y]
                weight = np.mean(pixel) / 255
                weight_array[x,y] = weight ** power
        
        self.weight_array = weight_array
        
    def _Create_Seed(self, res, threshold, image):
        while True:
            rand_x = np.random.rand() * res
            rand_y = np.random.rand() * res
            sampled_value = image.getpixel((rand_x,rand_y))
            if np.mean(sampled_value) < threshold:
                seed = np.array([rand_x,rand_y])
                return seed

    def Relax_Iteration(self, seeds):
        
        vor = Voronoi(seeds)
        res = self.image_resolution
        boundary_seeds = [
            np.array([-res, -res]),
            np.array([2 * res, -res]),
            np.array([2 * res, 2 * res]),
            np.array([-res, 2 * res])
        ]

        polygons = []
        open_cells = []
        for index, region in enumerate(vor.regions):
            poly = []
            if len(region) == 0:
                continue
            elif -1 in region:
                open_cells.append(region)
                continue
            for index in region:
                if index != -1:
                    poly.append(vor.vertices[index])
                else:
                    print("OUTSIDE")
            
            sorted_poly = Sort_Vertices(poly)

            polygons.append(sorted_poly)

        centroids = []
        for polygon in polygons:

            bbox = Raster_BBox(polygon, res)
            rastered_polygon = Scanline_Rasterize_Polygon(polygon, bbox, res)
            centroid = Weighted_Raster_Centroid(rastered_polygon, self.weight_array)
            if centroid:
                if centroid[0] < 0 or centroid[0] >= res or centroid[1] < 0 or centroid[1] >= res:
                    centroids.append(self._Create_Seed(res, 100, self.weight_image))
                else:
                    centroids.append(centroid)
            else:
                centroids.append(self._Create_Seed(res, 100, self.weight_image))
                
        
        new_seeds = boundary_seeds + centroids

        return new_seeds
    
    def Relax(self, iterations, **kwargs):
        seeds = self.seeds
        for i in range(iterations):

            new_seeds = self.Relax_Iteration(seeds)
            seeds = new_seeds
            print(f"Iteration {i}")

            if kwargs['save_iterations'] == True:
                self.Save_Result(f'sequence/iteration_{i}.jpg', seeds)

        self.relaxed_seeds = new_seeds
    
    def Save_Result(self, path, seeds):
        scale_factor = 2
        img = Image.new('RGB', (self.image_resolution *scale_factor, self.image_resolution *scale_factor))
        draw = ImageDraw.Draw(img)
        draw.rectangle([0,0,self.image_resolution *scale_factor, self.image_resolution *scale_factor],fill='white')
        r = 2
        for seed in seeds:
            x = (seed[0] + 0.5) * scale_factor
            y = (seed[1] + 0.5) * scale_factor
            draw.ellipse([x - r, y - r, x + r, y + r],fill='black')
        img.save(path)

if __name__ == "__main__":
    t0 = time.time()
    image_path = 'sampling/marilyn500_2.jpg'
    stippler = Stippler(image_path, power=2)
    stippler.Create_Seeds(15000,150)
    stippler.Relax(180, save_iterations=True)
    t1 = time.time()
    stippler.Save_Result(f'sequence/stipple_result.png', stippler.relaxed_seeds)
    elapsed = round(t1 - t0,2)
    print(f"Elapsed time: {elapsed} seconds")