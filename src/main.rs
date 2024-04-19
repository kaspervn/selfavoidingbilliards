use std::f64::consts::PI;
use std::iter::zip;
use simple_canvas::Canvas;
use rusty_ppm::ppm_writer;
use cgmath::{Vector3, Vector2};
use std::path::Path;
use std::sync::{Arc, Mutex};
use cgmath::num_traits::clamp;
use pointy::{Seg, Pt};
use heapless::Vec;
use bresenham;
use rand::prelude::*;
use rayon::prelude::*;

use enterpolation::{linear::ConstEquidistantLinear, Curve};
use palette::LinSrgb;

type SceneLinesType = Vec<Seg<f64>, 512>;
const ARENA_EDGES: usize = 4;
const ARENA_SIZE: f64 = 0.98;
const SAMPLES_PER_PIXEL: usize = 10;


fn draw_line<T: Copy>(canvas: &mut Canvas<T>, p0: bresenham::Point, p1: bresenham::Point, v: T)
{
    for (x, y) in bresenham::Bresenham::new(p0, p1) {
        let x = x as usize;
        let y = y as usize;
        if x < canvas.width && y < canvas.height {
            canvas.data[x + canvas.width * y] = v;
        }
    }
}

fn test_ball_with_scene(ball: Seg<f64>, scene : &SceneLinesType) -> Option<(Seg<f64>, Pt<f64>, f64)>
{
    let mut result: Option<(Seg<f64>, Pt<f64>, f64)> = None;

    for line in scene.iter() {

        match ball.intersection(*line) {
            Some(pt) => {
                let distance = pt.distance(ball.p0);
                let closest_distance_so_far = match result {
                    Some((_, __, x)) => {x}
                    None => {f64::INFINITY}
                };
                if distance < closest_distance_so_far {
                    result = Some((*line, pt, distance));
                }
            }
            None => {}
        }
    }

    result
}

fn reflection(ball: Pt<f64>, line: Seg<f64>, intersection: Pt<f64>) -> Seg<f64> {
    let c_line = line.p0 - intersection;
    let c_ball = ball - intersection;

    let x =  c_line.normalize() * (-c_ball.dot(c_line.normalize()));
    let reflected_dir = (x * 2.0 + c_ball).normalize();
    Seg::new(intersection, intersection + reflected_dir * 100.0)
}

fn draw_segment(canvas: &mut Canvas<Vector3<f64>>, segment: Seg<f64>)
{
    let p0 = ((segment.p0.x * canvas.width as f64) as isize, (segment.p0.y * canvas.height as f64) as isize);
    let p1 = ((segment.p1.x * canvas.width as f64) as isize, (segment.p1.y * canvas.height as f64) as isize);
    draw_line(canvas, p0, p1, Vector3::new(1.0, 1.0, 1.0));
}

#[derive(Clone)]
struct ThreadContext {
    rng: ThreadRng,
    canvas: Arc<Mutex<Canvas<f64>>>,
    lines: SceneLinesType,
    width: usize,
    height: usize
}

fn calc_pixel(context: &mut ThreadContext, pixel_idx: usize) {
    let pixel_loc = (pixel_idx % context.width, pixel_idx / context.width);
    let lines = &mut context.lines;
    let rng = &mut context.rng;

    let mut accumulated_pixel: f64 = 0.0;

    let start_x_table = (pixel_loc.0 as f64) / (context.width as f64);
    let start_y_table = (pixel_loc.1 as f64) / (context.height as f64);
    let start_pos = Pt::new(start_x_table, start_y_table);

    for _ in 0..SAMPLES_PER_PIXEL {
        lines.truncate(ARENA_EDGES);

        let rand_dir =  Pt::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)) * 10.0;
        let mut ball = Seg::new(start_pos, start_pos + rand_dir);
        let mut path_length: f64 = 0.0;
        let mut _no_bounces: i32 = 0;

        loop {
            match test_ball_with_scene(ball, &lines) {
                Some((line, col_point, distance)) => {
                    path_length += distance;
                    _no_bounces += 1;

                    if distance < 0.0001 || lines.is_full() {
                        accumulated_pixel += _no_bounces as f64;

                        break;
                    } else {
                        lines.push(Seg::new(ball.p0, col_point)).unwrap();
                        ball = reflection(ball.p0, line, col_point);

                        //Move ball forward a little bit to prevent immediate collision with itself
                        // or the line it just bounced of from
                        ball.p0 = ball.p0 + (ball.p1 - ball.p0).normalize() * 0.0001;
                    }
                }
                None => {
                    // keep pixel as background
                    return;
                }
            }
        }
    }


    context.canvas.lock().unwrap().data[pixel_loc.0 + context.width * pixel_loc.1] += accumulated_pixel;
}

fn initial_arena() -> SceneLinesType {
    let mut lines: SceneLinesType = Vec::new();

    for i in 0..ARENA_EDGES {
        let angle0 = (i as f64) * 2.0 * PI / (ARENA_EDGES as f64);
        let angle1 = ((i as f64) + 1.0) * 2.0 * PI / (ARENA_EDGES as f64);
        let center = Pt::new(0.5, 0.5);

        lines.push(Seg::new(center + Pt::from_angle(angle0) * ARENA_SIZE / 2.0,
                            center + Pt::from_angle(angle1) * ARENA_SIZE / 2.0)).unwrap();
    }

    lines
}

fn main() {
    println!("Hello, world!");

    let image_size = Vector2::new(1024, 1024);

    let mut canvas: Canvas<f64> = Canvas::new(image_size.x as usize, image_size.y as usize, 0.0);

    let width = canvas.width;
    let height = canvas.height;
    let shared_canvas = Arc::new(Mutex::new(canvas));


    let all_pixels = 0 .. width*height;
    let _: std::vec::Vec<_> = all_pixels.into_par_iter().map_init(|| ThreadContext{
        rng: thread_rng(),
        width,
        height,
        canvas: shared_canvas.clone(),
        lines: initial_arena(),
    }, calc_pixel).collect();


    println!("Writing image file");

    let canvas = shared_canvas.lock().unwrap();

    let gradient: std::vec::Vec<_> = ConstEquidistantLinear::<f32, _, 3>::equidistant_unchecked([
        LinSrgb::new(0.00, 0.05, 0.20),
        LinSrgb::new(0.70, 0.10, 0.20),
        LinSrgb::new(0.95, 0.90, 0.30),
    ]).take(256).collect();

    let mut post_processed_canvas: Canvas<Vector3<u8>> = Canvas::new(width, height, Vector3::new(0, 0, 0));
    let in_max = canvas.iter().max_by(|a, b| a.partial_cmp(&b).unwrap()).unwrap();

    for (a, b) in zip(canvas.iter(), post_processed_canvas.iter_mut()) {
        let x = clamp(254.0 * a/in_max, 0.0, 254.0) as u8;
        let cr = (gradient[x as usize].red * 255.0) as u8;
        let cg = (gradient[x as usize].green * 255.0) as u8;
        let cb = (gradient[x as usize].blue * 255.0) as u8;
        *b = Vector3::new(cr, cg, cb);

    }
    ppm_writer::write_binary_ppm(&post_processed_canvas, Path::new("."), "test.ppm").unwrap();
}
