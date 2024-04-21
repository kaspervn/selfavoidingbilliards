use std::f64::consts::PI;
use std::iter::zip;
use std::ops::Add;
use simple_canvas::Canvas;
use rusty_ppm::ppm_writer;
use cgmath::{Vector3, Vector2};
use std::path::Path;
use std::sync::{Arc, Mutex};
use cgmath::num_traits::clamp;
use heapless::Vec;
use bresenham;
use rand::prelude::*;
use rayon::prelude::*;
use geo::{Line, Coord, coord, Vector2DOps, EuclideanDistance};
use geo::line_intersection::{line_intersection, LineIntersection};

use enterpolation::{linear::ConstEquidistantLinear, Curve};
use palette::LinSrgb;

type SceneLinesType = Vec<Line, 100>;
const ARENA_EDGES: usize = 5;
const ARENA_SIZE: f64 = 0.98;
const SAMPLES_PER_PIXEL: usize = 2;


fn draw_line<T: Copy + Add<Output = T>>(canvas: &mut Canvas<T>, p0: bresenham::Point, p1: bresenham::Point, v: T)
{
    for (x, y) in bresenham::Bresenham::new(p0, p1) {
        let x = x as usize;
        let y = y as usize;
        if x < canvas.width && y < canvas.height {
            let idx = x + canvas.width * y;
            canvas.data[idx] = canvas.data[idx] + v;
        }
    }
}

fn draw_segment<T: Copy + Add<Output = T>>(canvas: &mut Canvas<T>, segment: Line, val: T)
{
    let p0 = ((segment.start.x * canvas.width as f64) as isize, (segment.start.y * canvas.height as f64) as isize);
    let p1 = ((segment.end.x * canvas.width as f64) as isize, (segment.end.y * canvas.height as f64) as isize);
    draw_line(canvas, p0, p1, val);
}

fn test_ball_with_scene(ball: Line, scene : &SceneLinesType) -> Option<(Line, Coord, f64)>
{
    let mut result: Option<(Line, Coord, f64)> = None;

    for line in scene {

        match line_intersection(*line, ball) {
            Some(LineIntersection::SinglePoint{intersection: pt, is_proper: _is_proper}) => {
                let distance = pt.euclidean_distance(&ball.start);
                let closest_distance_so_far = match result {
                    Some((_, __, x)) => {x}
                    None => {f64::INFINITY}
                };
                if distance < closest_distance_so_far {
                    result = Some((*line, pt, distance));
                }
            }
            _ => {}
        }
    }

    result
}

fn reflection(ball: Coord, line: Line, intersection: Coord) -> Option<Line> {
    let c_line = line.start - intersection;
    let c_ball = ball - intersection;

    let x =  c_line.try_normalize()? * (-c_ball.dot_product(c_line.try_normalize()?));
    let reflected_dir = (x * 2.0 + c_ball).try_normalize()?;
    Some(Line::new(intersection, intersection + reflected_dir * 100.0))
}

fn angled_coord(angle: f64) -> Coord {
    coord! {x: f64::cos(angle), y: f64::sin(angle)}
}

fn initial_arena() -> SceneLinesType {
    let mut obstacles: SceneLinesType = Vec::new();

    for i in 0..ARENA_EDGES {
        let angle0 = (i as f64) * 2.0 * PI / (ARENA_EDGES as f64);
        let angle1 = ((i as f64) + 1.0) * 2.0 * PI / (ARENA_EDGES as f64);
        let center = coord! {x: 0.5, y:0.5};

        obstacles.push(Line::new(center + angled_coord(angle0) * ARENA_SIZE / 2.0,
                                center + angled_coord(angle1) * ARENA_SIZE / 2.0)).unwrap();
    }

    obstacles
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
    let pixel_size = 1.0 / context.width as f64;

    let lines = &mut context.lines;
    let rng = &mut context.rng;

    let mut accumulated_pixel: f64 = 0.0;

    for iter_n in 0..SAMPLES_PER_PIXEL {
        lines.truncate(ARENA_EDGES);

        let start_x_table = (pixel_loc.0 as f64) / (context.width as f64) + rng.gen_range(0.0 .. pixel_size);
        let start_y_table = (pixel_loc.1 as f64) / (context.height as f64) + rng.gen_range(0.0 .. pixel_size);
        let start_pos = coord! {x: start_x_table, y: start_y_table};

        let rand_dir =  angled_coord(rng.gen_range(0.0 .. PI*2.0)) * 10.0;
        let mut ball = Line::new(start_pos, start_pos + rand_dir);
        let mut path_length: f64 = 0.0;
        let mut _no_bounces: i32 = 0;

        loop {
            match test_ball_with_scene(ball, &lines) {
                Some((line, col_point, distance)) => {
                    path_length += distance;
                    _no_bounces += 1;

                    if distance < 0.0001 || lines.is_full() {
                        accumulated_pixel += _no_bounces as f64;

                        let x = clamp((col_point.x * context.width as f64) as usize, 0, context.width - 1);
                        let y = clamp((col_point.y * context.height as f64) as usize, 0, context.height - 1);

                        context.canvas.lock().unwrap().data[x + context.width * y] += path_length;

                        break;
                    } else {
                        lines.push(Line::new(ball.start, col_point)).unwrap();
                        ball = reflection(ball.start, line, col_point).unwrap();

                        //Move ball forward a little bit to prevent immediate collision with itself
                        // or the line it just bounced of from
                        ball.start = ball.start + (ball.end - ball.start).try_normalize().unwrap() * 0.0001;
                    }
                }
                None => {
                    // keep pixel as background
                    // accumulated_pixel += _no_bounces as f64;
                    break;
                }
            }
        }

        // if draw_trajectory {
        //     let mut canvas = context.canvas.lock().unwrap();
        //
        //     for line in lines.iter() {
        //         println!("{}", line.start.x);
        //
        //         draw_segment(&mut canvas, *line, 10.0 * (iter_n+1) as f64);
        //     }
        // }
    }

     //


}



fn main() {
    println!("Hello, world!");

    let image_size = Vector2::new(512, 512);

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

    // calc_pixel(&mut ThreadContext{
    //     rng: thread_rng(),
    //     width,
    //     height,
    //     canvas: shared_canvas.clone(),
    //     lines: initial_arena(),
    // }, 200*512 + 200);


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
        let x = clamp(254.0 * a.log10()/in_max.log10(), 0.0, 254.0) as u8;
        let cr = (gradient[x as usize].red * 255.0) as u8;
        let cg = (gradient[x as usize].green * 255.0) as u8;
        let cb = (gradient[x as usize].blue * 255.0) as u8;
        // let x = clamp(255.0 * a, 0.0, 254.0) as u8;
        *b = Vector3::new(cr, cg, cb);

    }
    ppm_writer::write_binary_ppm(&post_processed_canvas, Path::new("."), "test.ppm").unwrap();
}
