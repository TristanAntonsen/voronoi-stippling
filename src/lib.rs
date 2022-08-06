use pyo3::prelude::*;

/// Formats the sum of two numbers as string.
#[pyfunction]
fn demo_function(a: usize, b: usize) -> PyResult<usize> {
    let result = a * b;
    Ok(result)
}

/// Calculates weighted centroid of pixels.
/// pixels -> list (nested) [[x,y],[x,y]...]
/// weights -> list (nested) [resolution, resolution]
#[pyfunction]
fn weighted_raster_centroid(pixels: Vec<[f32; 2]>, weights: Vec<[f32; 1080]>) -> PyResult<(f32, f32)> {

    let pixel_count = pixels.len();
    // let increment_size = round::ceil(pixel_count);
    // let increment_size = (pixel_count as f32 / 500.0).ceil();
    
    let mut pixel;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut sample_x;
    let mut sample_y;
    let mut sampled_value;
    let mut weight;
    let mut total_weight = 0.0;


    let mut i = 0;
    while i < pixel_count {
        pixel = &pixels[i];

        sample_x = pixel[1] as usize;
        sample_y = pixel[0] as usize;
        sampled_value = weights[sample_x][sample_y];

        weight = 1.0 - sampled_value;
        total_weight += weight;

        cx += pixel[0] * weight;
        cy += pixel[1] * weight;

        i += 1;
    }

    if total_weight == 0.0 {
        total_weight = pixel_count as f32;
    }

    cx /= total_weight;
    cy /= total_weight;

    let result;

    if pixel_count == 0 {
        result = (0.0,0.0);
    } else if cx == 0.0 {
        result = (0.0,0.0);
    } else if cy == 0.0 {
        result = (0.0,0.0);
    } else {
        result = (cx, cy);
    }

    Ok(result)

}



/// A Python module implemented in Rust.
#[pymodule]
fn voronoi_stippling(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(demo_function, m)?)?;
    m.add_function(wrap_pyfunction!(weighted_raster_centroid, m)?)?;
    Ok(())
}