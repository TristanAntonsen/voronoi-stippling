from PIL import Image, ImageDraw, ImageFilter
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np
from geometry import Sort_Vertices
from rasterize import Scanline_Rasterize_Polygon, Raster_BBox, Weighted_Raster_Centroid

class Stippler:
    def __init__(self, image_path):
        
        self.weight_image = Image.open(image_path)
        self.image_resolution = self.weight_image.size[0]
        self._Pixel_Array()

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

        self.seeds = seeds

    def _Pixel_Array(self):
        image_array = np.array(self.weight_image)
        weight_array = np.zeros([self.image_resolution,self.image_resolution])

        for x in range(self.image_resolution):
            for y in range(self.image_resolution):
                pixel = image_array[x,y]
                weight_array[x,y] = (np.mean(pixel) / 255)
        
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
    
    def Relax(self, iterations):
        seeds = self.seeds
        for i in range(iterations):

            new_seeds = self.Relax_Iteration(seeds)
            seeds = new_seeds
            print(f"Iteration {i}")

        self.relaxed_seeds = new_seeds
    
    def Save_Result(self, path):
        img = Image.new('RGB', (self.image_resolution, self.image_resolution))
        draw = ImageDraw.Draw(img)
        draw.rectangle([0,0,self.image_resolution, self.image_resolution],fill='white')
        r = 5
        for seed in self.relaxed_seeds:
            draw.ellipse([seed[0] - r, seed[1] - r, seed[0] + r, seed[1] + r],fill='black')
        img.save(path)

if __name__ == "__main__":
    image_path = 'sampling/test_im_500.png'
    stippler = Stippler(image_path)
    stippler.Create_Seeds(100,150)
    stippler.Relax(30)
    stippler.Save_Result(f'sequence/stipple_result.png')
