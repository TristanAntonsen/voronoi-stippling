
use pyo3::prelude::*;

/// Formats the sum of two numbers as string.
#[pyfunction]
fn demo_function(a: usize, b: usize) -> PyResult<usize> {
    let result = a * b;
    Ok(result)
}

/// A Python module implemented in Rust.
#[pymodule]
fn voronoi_stippling(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(demo_function, m)?)?;
    Ok(())
}