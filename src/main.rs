use std::f64::consts::PI;
use std::iter::zip;
use std::ops::{Add, AddAssign};
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::sync::mpsc;
use std::sync::mpsc::RecvTimeoutError;
use std::thread;
use std::time::Duration;

use bresenham;
use cgmath::Vector3;
use cgmath::num_traits::clamp;
use enterpolation::{Curve, linear::ConstEquidistantLinear};
use geo::{Coord, coord, EuclideanDistance, Line, Vector2DOps};
use geo::line_intersection::{line_intersection, LineIntersection};
use heapless::Vec;
use indicatif::{ProgressBar, ProgressStyle};
use palette::LinSrgb;
use rand::prelude::*;
use rusty_ppm::ppm_writer;
use simple_canvas::Canvas;
use tiff;
use std::fs::File;
use std::process::Output;
use rusty_ppm::utils::generate_sample_binary_image;
use tiff::encoder::colortype;

use chrono::prelude::*;

use crate::FromThreadMsg::REPORT;
use crate::ToThreadMsg::{ACCUMULATE, STOP};

type SceneLinesType = Vec<Line, 100>;
const ARENA_EDGES: usize = 5;
const ARENA_SIZE: f64 = 0.98;


fn _draw_line<T: Copy + Add<Output = T>>(canvas: &mut Canvas<T>, p0: bresenham::Point, p1: bresenham::Point, v: T)
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

fn _draw_segment<T: Copy + Add<Output = T>>(canvas: &mut Canvas<T>, segment: Line, val: T)
{
    let p0 = ((segment.start.x * canvas.width as f64) as isize, (segment.start.y * canvas.height as f64) as isize);
    let p1 = ((segment.end.x * canvas.width as f64) as isize, (segment.end.y * canvas.height as f64) as isize);
    _draw_line(canvas, p0, p1, val);
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

type ShaderFunc<T> = fn(start_pos: Coord, path_length: f64, no_bounces: usize) -> T;

fn single_simulation<T: AddAssign>(canvas: &mut Canvas<T>,
                        obstacles: &mut SceneLinesType,
                        rng: &mut ThreadRng,
                        canvas_shader: ShaderFunc<T>
)
{
    let clean_scene_size = obstacles.len();

    let start_pos = coord! {x: rng.gen_range(0.0 .. 1.0),
                            y: rng.gen_range(0.0 .. 1.0)};
    let rand_dir =  angled_coord(rng.gen_range(0.0 .. PI*2.0)) * 10.0;

    let mut ball = Line::new(start_pos, start_pos + rand_dir);
    let mut path_length: f64 = 0.0;
    let mut _no_bounces: usize = 0;

    loop {
        match test_ball_with_scene(ball, &obstacles) {
            Some((line, col_point, distance)) => {
                path_length += distance;
                _no_bounces += 1;

                if distance < 0.0001 || obstacles.is_full() {

                    let x = clamp(f64::round(col_point.x * canvas.width as f64) as usize, 0, canvas.width - 1);
                    let y = clamp(f64::round(col_point.y * canvas.height as f64) as usize, 0, canvas.height - 1);

                    canvas.data[x + canvas.width * y] += canvas_shader(start_pos, path_length, _no_bounces);

                    break;
                } else {
                    obstacles.push(Line::new(ball.start, col_point)).unwrap();

                    match reflection(ball.start, line, col_point) {
                        None => {
                            let x = clamp(f64::round(col_point.x * canvas.width as f64) as usize, 0, canvas.width - 1);
                            let y = clamp(f64::round(col_point.y * canvas.height as f64) as usize, 0, canvas.height - 1);

                            canvas.data[x + canvas.width * y] += canvas_shader(start_pos, path_length, _no_bounces);
                        }
                        Some(b) => {ball = b;}
                    }

                    // Move ball forward a little bit to prevent immediate collision with itself
                    // or the line it just bounced of from
                    ball.start = ball.start + (ball.end - ball.start).try_normalize().unwrap() * 0.0001;
                }
            }
            None => {
                break;
            }
        }
    }

    // Leave the scene in state that we started with
    obstacles.truncate(clean_scene_size);
}

enum ToThreadMsg {
    ACCUMULATE,
    STOP
}

enum FromThreadMsg {
    REPORT(usize)
}

fn sim_thread<T: AddAssign + Default + Clone>(rx: mpsc::Receiver<ToThreadMsg>,
              tx: mpsc::Sender<FromThreadMsg>,
              result_canvas: Arc<Mutex<Canvas<T>>>,
              shader_func: ShaderFunc<T>)
{
    const NUMBER_OF_SIMS_PER_REPORT: usize = 100_000;

    let width = result_canvas.lock().unwrap().width;
    let height = result_canvas.lock().unwrap().height;

    let mut thread_canvas: Canvas<T> = Canvas::new(width, height, T::default());
    let mut scene = initial_arena();
    let mut rng = thread_rng();

    loop {

        for _ in 0..NUMBER_OF_SIMS_PER_REPORT {
            single_simulation(&mut thread_canvas, &mut scene, &mut rng, shader_func);
        }
        tx.send(REPORT(NUMBER_OF_SIMS_PER_REPORT)).unwrap();

        match rx.recv_timeout(Duration::ZERO) {
            Ok(ACCUMULATE) => {
                let mut locked_canvas = result_canvas.lock().unwrap();
                for (p_in, p_out) in zip(thread_canvas.iter(), locked_canvas.iter_mut()) {
                    *p_out += p_in.clone();
                }
            }
            Ok(STOP) => {
                return
            }
            Err(RecvTimeoutError::Disconnected) => {
                panic!();
            }
            _ => {}
        }
    }
}

#[derive(Debug)]
struct ThreadHandle {
    handle: thread::JoinHandle<()>,
    to_thread: mpsc::Sender<ToThreadMsg>,
    from_thread: mpsc::Receiver<FromThreadMsg>,
}


fn main() {
    let canvas: Canvas<f64> = Canvas::new(512, 512, 0.0);

    let shared_canvas = Arc::new(Mutex::new(canvas));

    let shader: ShaderFunc<_> = |_start_pos: Coord, path_length: f64, _no_bounces: usize| path_length;

    const MAX_THREADS: usize = 25;

    let mut thread_handles: Vec<ThreadHandle, MAX_THREADS> = Vec::new();

    for _ in 0..MAX_THREADS {
        let (to_thread, to_thread_rx) = mpsc::channel();
        let (from_thread_tx, from_thread) = mpsc::channel();


        let canvas_ref = shared_canvas.clone();

        let thread_handle = thread::spawn(move || {
            sim_thread(to_thread_rx, from_thread_tx, canvas_ref, shader)
        });

        thread_handles.push(ThreadHandle {
            handle: thread_handle,
            to_thread,
            from_thread,
        }).unwrap();
    }

    //let total_simulations: usize = 1_000_000_000;
    let total_simulations: usize = 20_000_000;

    let bar = ProgressBar::new(total_simulations as u64);
    bar.set_style(ProgressStyle::with_template("[{elapsed}]/[{eta} left] {bar:40.cyan/blue} {percent}% {pos:>7}/{len:7} {per_sec}").unwrap());

    let mut simulations_done: usize = 0;
    while simulations_done < total_simulations {
        for thread in &thread_handles {
            assert!(!thread.handle.is_finished());

            match thread.from_thread.recv_timeout(Duration::from_millis(1)) {
                Ok(REPORT(n)) => {
                    simulations_done += n;
                    bar.inc(n as u64);
                    bar.eta();
                }
                Err(RecvTimeoutError::Disconnected) => {
                    panic!();
                }

                Err(RecvTimeoutError::Timeout) => {}
            }
        }
    }
    bar.finish();

    for thread in &thread_handles {
        thread.to_thread.send(ACCUMULATE).unwrap();
        thread.to_thread.send(STOP).unwrap();
    }

    while ! thread_handles.is_empty() {
        let handle = thread_handles.pop().unwrap();
        handle.handle.join().unwrap();
    }

    println!("post processing image file");

    let canvas = shared_canvas.lock().unwrap();

    let mut normalized_canvas: Canvas<u32> = Canvas::new(canvas.width, canvas.height, 0);
    let in_max = canvas.iter().max_by(|a, b| a.partial_cmp(&b).unwrap()).unwrap();
    for (a, b) in zip(canvas.iter(), normalized_canvas.iter_mut()) {
        *b = clamp((u32::MAX as f64 * a.log10() / in_max.log10()) as u32, 0, u32::MAX);
    }

    let f = File::create(format!("raw-{}.tiff", Local::now())).unwrap();

    let mut encoder = tiff::encoder::TiffEncoder::new(f).unwrap();
    encoder.write_image::<colortype::Gray32>(normalized_canvas.width as u32, normalized_canvas.height as u32, &normalized_canvas.data).unwrap();

    // let mut image = encoder.new_image::<colortype::RGB8>(canvas.width as u32, canvas.height as u32).unwrap();
    // let image_data = normalized_canvas.data;
    // image.encoder().write_data(image_data).unwrap();
    // image.finish().unwrap();

    /*let gradient: std::vec::Vec<_> = ConstEquidistantLinear::<f32, _, 3>::equidistant_unchecked([
        LinSrgb::new(0.00, 0.05, 0.20),
        LinSrgb::new(0.70, 0.10, 0.20),
        LinSrgb::new(0.95, 0.90, 0.30),
    ]).take(256).collect();

    let mut post_processed_canvas: Canvas<Vector3<u8>> = Canvas::new(canvas.width, canvas.height, Vector3::new(0, 0, 0));
    let in_max = canvas.iter().max_by(|a, b| a.partial_cmp(&b).unwrap()).unwrap();

    for (a, b) in zip(canvas.iter(), post_processed_canvas.iter_mut()) {
        let x = clamp(254.0 * a.log10()/in_max.log10(), 0.0, 254.0) as u8;
        let cr = (gradient[x as usize].red * 255.0) as u8;
        let cg = (gradient[x as usize].green * 255.0) as u8;
        let cb = (gradient[x as usize].blue * 255.0) as u8;
        // let x = clamp(255.0 * a, 0.0, 254.0) as u8;
        *b = Vector3::new(cr, cg, cb);

    }

    println!("writing image file");
    ppm_writer::write_binary_ppm(&post_processed_canvas, Path::new("."), "test.ppm").unwrap();*/
}
