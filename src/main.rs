use simple_canvas::Canvas;
use rusty_ppm::ppm_writer;
use cgmath::{Vector3, Vector2};
use std::path::Path;

fn main() {
    println!("Hello, world!");

    let image_size = Vector2::new(1024, 1024);

    let canvas: Canvas<Vector3<u8>> = Canvas::new(image_size.x as usize, image_size.y as usize, Vector3::new(0, 0, 0));


    ppm_writer::write_binary_ppm(&canvas, Path::new("."), "test.ppm").unwrap();
}
