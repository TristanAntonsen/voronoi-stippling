fn main() {
    println!("Hello World!");
    let test = vec![[1,2],[1,2]];

    print_type_of(&test);
}

fn print_type_of<T>(_: &T) {
    println!("{}", std::any::type_name::<T>())
}