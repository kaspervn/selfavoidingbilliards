use std::f64::consts::PI;
use std::iter::zip;
use simple_canvas::Canvas;
use rusty_ppm::ppm_writer;
use cgmath::{Vector3, Vector2};
use std::path::Path;
use cgmath::num_traits::clamp;
use pointy::{Seg, Pt};
use heapless::Vec;
use bresenham;

type SceneLinesType = Vec<Seg<f64>, 1024>;

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

fn test_ball_with_scene(ball: Seg<f64>, scene : &SceneLinesType, ignore_line: Option<usize>) -> Option<(Seg<f64>, Pt<f64>, f64, usize)>
{
    let mut result: Option<(Seg<f64>, Pt<f64>, f64, usize)> = None;

    for (n, line) in scene.iter().enumerate() {
        if ignore_line.unwrap_or(usize::MAX) == n {
            continue;
        }

        match ball.intersection(*line) {
            Some(pt) => {
                let distance = pt.distance(ball.p0);
                let closest_distance_so_far = match result {
                    Some((_, __, x, ___)) => {x}
                    None => {f64::INFINITY}
                };
                if distance < closest_distance_so_far {
                    result = Some((*line, pt, distance, n));
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

fn main() {
    println!("Hello, world!");

    let image_size = Vector2::new(1024, 1024);

    let mut canvas: Canvas<Vector3<f64>> = Canvas::new(image_size.x as usize, image_size.y as usize, Vector3::new(0.0, 0.0, 0.0));

    let mut lines: SceneLinesType = Vec::new();

    //
    // lines.push(Seg::new(Pt::new(0.20, 0.0), Pt::new(0.6, 0.5))).unwrap();
    //
    // for line in &lines {
    //     draw_segment(&mut canvas, *line);
    // }
    //
    // let b = Seg::new(Pt::new(1.0, 0.0), Pt::new(-100.0, 100.0));
    // draw_segment(&mut canvas, b);
    //
    // match(test_ball_with_scene(b, &lines))
    // {
    //     None => {}
    //     Some((line, col_point, distance)) => {
    //         let r =  reflection(b.p0, line, col_point, distance);
    //         draw_segment(&mut canvas, r);
    //     }
    // }



    let n = 3;
    let arena_size: f64 = 0.8;

    for start_x_img in 0 .. canvas.width {
        println!("{}", start_x_img);
        for start_y_img in 0 .. canvas.height {


            // if start_x_img != 500 || start_y_img != 512 {
            //     continue;
            // }

            lines.clear();

            for i in 0 .. n {
                let angle0 = (i as f64) * 2.0 * PI / (n as f64);
                let angle1 = ((i as f64) + 1.0) * 2.0 * PI / (n as f64);
                let center = Pt::new(0.5, 0.5);

                lines.push(Seg::new(center + Pt::from_angle(angle0) * arena_size / 2.0,
                                    center + Pt::from_angle(angle1) * arena_size / 2.0)).unwrap();
            }
            //
            // lines.push(Seg::new(Pt::new(0.1, 0.1), Pt::new(0.9, 0.11))).unwrap();
            // lines.push(Seg::new(Pt::new(0.9,0.11), Pt::new(0.91, 0.9))).unwrap();
            // lines.push(Seg::new(Pt::new(0.91,0.9), Pt::new(0.1, 0.1))).unwrap();

            let start_x_table = (start_x_img as f64) / (canvas.width as f64);
            let start_y_table = (start_y_img as f64) / (canvas.height as f64);

            let start_table = Pt::new(start_x_table, start_y_table);

            let mut ball = Seg::new(start_table, start_table - (start_table - Pt::new(0.5, 0.5))*10.0 );
            let mut last_intersected_line: Option<usize> = None;
            let mut _total_length: f64 = 0.0;
            let mut total_bounces: f64 = 0.0;

            loop {
                match test_ball_with_scene(ball, &lines, last_intersected_line) {

                    Some((line, col_point, distance, collided_with_index)) => {
                        last_intersected_line = Some(collided_with_index);
                        _total_length += distance;
                        total_bounces += 1.0;

                        if distance < 0.0001 {
                            // enough tested, color in the pixel
                            //println!("{}", total_length);
                            canvas.data[start_x_img + canvas.width * start_y_img] = Vector3::new(_total_length.log10(), 0.0, 0.0);

                            break;
                        } else {
                            if lines.push(Seg::new(ball.p0, col_point)).is_err() {
                                canvas.data[start_x_img + canvas.width * start_y_img] = Vector3::new(_total_length.log10(), 0.0, 0.0);
                                break;
                            }
                            ball = reflection(ball.p0, line, col_point);
                            ball.p0 = ball.p0 + (ball.p1 - ball.p0).normalize() * 0.0001;
                            //draw_segment(&mut canvas, ball);
                            // for line in &lines {
                            //     draw_segment(&mut canvas, *line);
                            // }
                        }

                    }
                    None => {
                        // keep pixel as background
                        break;
                    }
                }
            }
        }
    }


    println!("Writing image file");
    let mut post_processed_canvas: Canvas<Vector3<u8>> = Canvas::new(canvas.width, canvas.height, Vector3::new(0, 0, 0));
    let in_max = canvas.iter().max_by(|a, b| a.x.partial_cmp(&b.x).unwrap()).unwrap();

    for (a, b) in zip(canvas.iter(), post_processed_canvas.iter_mut()) {
        let x = clamp(254.0 * a.x/in_max.x, 0.0, 254.0) as u8;
        *b = Vector3::new(x, x, x);

    }
    ppm_writer::write_binary_ppm(&post_processed_canvas, Path::new("."), "test.ppm").unwrap();
}
