use std::f64::consts::PI;
use std::iter::zip;
use std::ops::{Add, AddAssign};
use std::sync::{Arc, Mutex};
use std::sync::mpsc;
use std::sync::mpsc::RecvTimeoutError;
use std::thread;
use std::time::Duration;

use bresenham;
use cgmath::num_traits::clamp;
use geo::{Coord, coord, EuclideanDistance, Line, Vector2DOps};
use geo::line_intersection::{line_intersection, LineIntersection};
use heapless::Vec;
use indicatif::{ProgressBar, ProgressStyle};
use rand::prelude::*;
use simple_canvas::Canvas;
use tiff;
use std::fs::File;
use tiff::encoder::colortype;

use chrono::prelude::*;

use crate::FromThreadMsg::REPORT;
use crate::ToThreadMsg::{ACCUMULATE, STOP};

type ShaderFunc<T> = fn(start_pos: Coord, path_length: f64, no_bounces: usize) -> T;
type Obsctacles = Vec<Line, MAX_NO_OBSTACLES>;

const MAX_NO_OBSTACLES: usize = 200;
const ARENA_EDGES: usize = 17;
const ARENA_SIZE: f64 = 0.98;                       // size of arena, as ratio of the whole image
const IMAGE_SIZE: usize = 512;                      // width and height in pixels
const MIN_NUM_OF_SIMULATIONS: usize = 10_000_000;   // Minimum number of simulations to do, should not be much more

const SHADER_FUNC: ShaderFunc<f64> = |_start_pos: Coord, path_length: f64, _no_bounces: usize| path_length;

const NO_THREADS: usize = 25;


fn initial_arena() -> Obsctacles
{
    let mut obstacles: Obsctacles = Vec::new();

    for i in 0..ARENA_EDGES {
        let angle0 = (i as f64)         * 2.0 * PI / (ARENA_EDGES as f64);
        let angle1 = ((i as f64) + 1.0) * 2.0 * PI / (ARENA_EDGES as f64);
        let center = coord! {x: 0.5, y:0.5};

        obstacles.push(Line::new(center + angle(angle0) * ARENA_SIZE / 2.0,
                                 center + angle(angle1) * ARENA_SIZE / 2.0)).unwrap();
    }

    obstacles
}


fn test_ball_with_obstacles(ball: Line, obstacles: &Obsctacles) -> Option<(Line, Coord, f64)>
{
    let mut result: Option<(Line, Coord, f64)> = None;

    for line in obstacles {

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


fn reflection(ball: Coord, line: Line, intersection: Coord) -> Option<Line>
{
    let centered_line_endpoint = line.start - intersection;
    let centered_ball = ball - intersection;

    let x =  centered_line_endpoint.try_normalize()? * (-centered_ball.dot_product(centered_line_endpoint.try_normalize()?));
    let reflected_dir = (x * 2.0 + centered_ball).try_normalize()?;

    // Move ball forward a little bit to prevent immediate collision with itself
    // or the line it just bounced of from
    //                            VVVVVVVVVVVVVVVVVVVVVV
    Some(Line::new(intersection + reflected_dir * 0.0001 , intersection + reflected_dir * 10.0))
}


enum SimStepOutcome {
    Trapped(Coord),
    Bounced,
    Escaped         // probably started outside already
}


fn single_simulation<T: AddAssign>(canvas: &mut Canvas<T>,
                                   obstacles: &mut Obsctacles,
                                   rng: &mut ThreadRng,
                                   canvas_shader: ShaderFunc<T>)
{
    let clean_scene_size = obstacles.len();

    let start_pos = coord! {x: rng.gen_range(0.0 .. 1.0),
                            y: rng.gen_range(0.0 .. 1.0)};
    let rand_dir =  angle(rng.gen_range(0.0 .. PI*2.0)) * 10.0;

    let mut ball = Line::new(start_pos, start_pos + rand_dir);
    let mut path_length: f64 = 0.0;
    let mut no_bounces: usize = 0;

    loop {
        let step_outcome: SimStepOutcome = match test_ball_with_obstacles(ball, &obstacles) {

            Some((line, col_point, distance)) => {
                path_length += distance;
                no_bounces += 1;

                if distance < 0.0001 || obstacles.is_full() {
                    SimStepOutcome::Trapped(col_point) // trapped
                } else {
                    obstacles.push(Line::new(ball.start, col_point)).unwrap();

                    match reflection(ball.start, line, col_point) {
                        Some(b) => {
                            ball = b;
                            SimStepOutcome::Bounced // continue bouncing
                        }

                        // reflection calculation failed
                        None => {
                            SimStepOutcome::Trapped(col_point) // trapped
                        }
                    }
                }
            }

            // no collision, it must have escaped, (or more likely, it started outside)
            None => {
                SimStepOutcome::Escaped
            }
        };

        match step_outcome {
            SimStepOutcome::Trapped(pt) => {
                let x = clamp(f64::round(pt.x * canvas.width as f64) as usize, 0, canvas.width - 1);
                let y = clamp(f64::round(pt.y * canvas.height as f64) as usize, 0, canvas.height - 1);

                canvas.data[x + canvas.width * y] += canvas_shader(start_pos, path_length, no_bounces);
                break;
            }
            SimStepOutcome::Bounced => {
                // keep looping
            }
            SimStepOutcome::Escaped => {
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
    join_handle: thread::JoinHandle<()>,
    to_thread: mpsc::Sender<ToThreadMsg>,
    from_thread: mpsc::Receiver<FromThreadMsg>,
}


fn main()
{
    let canvas: Canvas<f64> = Canvas::new(IMAGE_SIZE, IMAGE_SIZE, 0.0);
    let shared_canvas = Arc::new(Mutex::new(canvas));

    // Start all threads
    let mut thread_handles: Vec<ThreadHandle, NO_THREADS> = Vec::new();
    for _ in 0..NO_THREADS {
        let (to_thread, to_thread_rx) = mpsc::channel();
        let (from_thread_tx, from_thread) = mpsc::channel();

        let canvas_ref = shared_canvas.clone();

        thread_handles.push(ThreadHandle {
            join_handle: thread::spawn(move || {
                sim_thread(to_thread_rx, from_thread_tx, canvas_ref, SHADER_FUNC)
            }),
            to_thread,
            from_thread,
        }).unwrap();
    }

    // Keep track of the progress of all threads and report with a nice progress bar
    let progbar = ProgressBar::new(MIN_NUM_OF_SIMULATIONS as u64);
    progbar.set_style(ProgressStyle::with_template("[{elapsed}]/[{eta} left] {bar:40.cyan/blue} {percent}% {pos:>7}/{len:7} {per_sec}").unwrap());

    let mut simulations_done: usize = 0;
    while simulations_done < MIN_NUM_OF_SIMULATIONS {
        for thread in &thread_handles {
            assert!(!thread.join_handle.is_finished());

            match thread.from_thread.recv_timeout(Duration::from_millis(1)) {
                Ok(REPORT(n)) => {
                    simulations_done += n;
                    progbar.inc(n as u64);
                }
                Err(RecvTimeoutError::Disconnected) => {
                    panic!();
                }

                Err(RecvTimeoutError::Timeout) => {}
            }
        }
    }
    progbar.finish();


    // Asks all threads to accumulate in the shared canvas and ask them to stop working
    for thread in &thread_handles {
        thread.to_thread.send(ACCUMULATE).unwrap();
        thread.to_thread.send(STOP).unwrap();
    }

    // Join all threads
    while ! thread_handles.is_empty() {
        let handle = thread_handles.pop().unwrap();
        handle.join_handle.join().unwrap();
    }

    // Make a new canvas, normalized and scaled to u32::MAX
    let canvas = shared_canvas.lock().unwrap();
    let mut normalized_canvas: Canvas<u32> = Canvas::new(canvas.width, canvas.height, 0);
    let src_max = canvas.iter().max_by(|a, b| a.partial_cmp(&b).unwrap()).unwrap();
    for (src, target) in zip(canvas.iter(), normalized_canvas.iter_mut()) {
        *target = clamp((u32::MAX as f64 * src.log10() / src_max.log10()) as u32, 0, u32::MAX);
    }

    // Write a 32bit grayscale tiff
    let f = File::create(format!("raw-{}.tiff", Local::now())).unwrap();
    let mut encoder = tiff::encoder::TiffEncoder::new(f).unwrap();
    encoder.write_image::<colortype::Gray32>(normalized_canvas.width as u32,
                                             normalized_canvas.height as u32,
                                             &normalized_canvas.data).unwrap();
}


fn angle(angle: f64) -> Coord
{
    coord! {x: f64::cos(angle), y: f64::sin(angle)}
}


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
