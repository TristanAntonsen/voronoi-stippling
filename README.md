# Weighted voronoi point stippling

## Activating virtual environment:

(Windows)

    venv\Scripts\activate

(Unix)

    source venv/bin/activate

Compiling rust library:

    maturin devlop

or

    maturin develop --release

## Running rust tests

    cargo run --bin tests

## Python usage of Rust library
    >>> import voronoi_stippling as vs
    >>>
    >>> vs.demo_function(10,10) # multiplies arguments (rust)
    100