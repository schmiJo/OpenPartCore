//! A rendered picture has to mean something, so these measure what's on it against the geometry.

use coreshape::*;
use coreshape_render::*;

fn is_background(rgb: &[u8]) -> bool {
    // Close to the page colour, which covers edge pixels blended with it only partly.
    rgb[0] > 235 && rgb[1] > 235 && rgb[2] > 230
}

/// The body's share of the picture, ignoring the corner with the axes.
fn covered(image: &Image) -> usize {
    let (w, h) = (image.width as usize, image.height as usize);
    let mut n = 0;
    for y in 0..h {
        for x in 0..w {
            if x < 70 && y + 70 > h {
                continue;
            }
            if !is_background(&image.rgb[(y * w + x) * 3..(y * w + x) * 3 + 3]) {
                n += 1;
            }
        }
    }
    n
}

fn at(image: &Image, x: usize, y: usize) -> [u8; 3] {
    let i = (y * image.width as usize + x) * 3;
    [image.rgb[i], image.rgb[i + 1], image.rgb[i + 2]]
}

fn options(view: View) -> Options {
    Options { width: 400, height: 400, view, ..Options::default() }
}

#[test]
fn a_sphere_covers_the_area_of_its_circle() {
    let body = sphere(&Frame::WORLD, 10.0, 1).unwrap();
    let image = render(&body, &options(View::Iso)).unwrap();
    let r = 10.0 / image.mm_per_pixel;
    let want = PI * r * r;
    let got = covered(&image) as f64;
    assert!((got - want).abs() / want < 0.03, "{got} pixels, want {want}");
}

#[test]
fn a_box_seen_from_the_top_covers_its_rectangle() {
    let body = box_solid(&Frame::WORLD, 30.0, 10.0, 5.0, 1).unwrap();
    let image = render(&body, &options(View::Top)).unwrap();
    let want = (30.0 / image.mm_per_pixel) * (10.0 / image.mm_per_pixel);
    let got = covered(&image) as f64;
    assert!((got - want).abs() / want < 0.03, "{got} pixels, want {want}");
}

#[test]
fn views_look_from_where_they_say() {
    // A box with a tall tower at +x. From the front (looking along +y) the tower is on the right.
    let base = box_solid(&Frame::translation(v3(-20.0, -5.0, 0.0)), 40.0, 10.0, 4.0, 1).unwrap();
    let tower = box_solid(&Frame::translation(v3(10.0, -5.0, 4.0)), 10.0, 10.0, 20.0, 2).unwrap();
    let body = union(&base, &tower).unwrap();
    let occupied = |image: &Image, x: usize, y: usize| !is_background(&at(image, x, y).map(|c| c).as_slice());
    let front = render(&body, &options(View::Front)).unwrap();
    let scale = 1.0 / front.mm_per_pixel;
    let (cx, cy) = (200.0, 200.0);
    // The picture is centred on the bounding box: x from -20 to 20, z from 0 to 24.
    let tower_px = ((cx + 15.0 * scale) as usize, (cy - 8.0 * scale) as usize);
    let empty_px = ((cx - 15.0 * scale) as usize, (cy - 8.0 * scale) as usize);
    assert!(occupied(&front, tower_px.0, tower_px.1), "the tower shows on the right from the front");
    assert!(!occupied(&front, empty_px.0, empty_px.1), "and nothing on the left");
    // From the back it's the other way round.
    let back = render(&body, &options(View::Back)).unwrap();
    assert!(occupied(&back, empty_px.0, empty_px.1));
    assert!(!occupied(&back, tower_px.0, tower_px.1));
    // From above, the long side runs across.
    let top = render(&body, &options(View::Top)).unwrap();
    let s = 1.0 / top.mm_per_pixel;
    assert!(occupied(&top, (cx + 18.0 * s) as usize, 200));
    assert!(!occupied(&top, 200, (cy - 8.0 * s) as usize));
}

#[test]
fn what_is_in_front_hides_what_is_behind() {
    // Two boxes in a line along the view: only the near one's faces can show.
    let near = box_solid(&Frame::translation(v3(-10.0, -30.0, -10.0)), 20.0, 5.0, 20.0, 1).unwrap();
    let far = box_solid(&Frame::translation(v3(-4.0, 20.0, -4.0)), 8.0, 5.0, 8.0, 2).unwrap();
    let body = union(&near, &far).unwrap();
    let image = render(&body, &options(View::Front)).unwrap();
    let seen: Vec<u32> = image.visible_faces.iter().map(|(f, _)| body.faces[*f].tag.feature).collect();
    assert!(seen.contains(&1));
    assert!(!seen.contains(&2), "the far box is completely hidden: {seen:?}");
}

#[test]
fn faces_seen_are_the_ones_facing_the_camera() {
    let body = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    let top = render(&body, &options(View::Top)).unwrap();
    assert_eq!(top.visible_faces.len(), 1, "{:?}", top.visible_faces);
    let iso = render(&body, &options(View::Iso)).unwrap();
    assert_eq!(iso.visible_faces.len(), 3, "{:?}", iso.visible_faces);
}

#[test]
fn highlights_show_in_their_colour() {
    let body = box_solid(&Frame::WORLD, 10.0, 10.0, 10.0, 1).unwrap();
    let top_face = body.faces.iter().position(|f| f.tag == FaceTag { feature: 1, slot: 1 }).unwrap();
    let plain = render(&body, &options(View::Top)).unwrap();
    let marked = render(&body, &Options { highlight_faces: vec![top_face], ..options(View::Top) }).unwrap();
    let (p, m) = (at(&plain, 200, 200), at(&marked, 200, 200));
    assert!(m[0] > p[0] + 30 && m[2] + 30 < p[2], "{p:?} became {m:?}");
}

#[test]
fn labels_land_on_the_faces_they_name() {
    let body = box_solid(&Frame::WORLD, 40.0, 30.0, 20.0, 1).unwrap();
    let names: Vec<String> = (0..body.faces.len()).map(|f| format!("face.{f}")).collect();
    let image = render(&body, &Options { face_names: names.clone(), ..options(View::Iso) }).unwrap();
    assert_eq!(image.labels.len(), 3, "{:?}", image.labels);
    for l in &image.labels {
        assert!(names.contains(&l.text));
    }
    // Far apart: each sits inside its own face.
    for (i, a) in image.labels.iter().enumerate() {
        for b in &image.labels[..i] {
            let d = ((a.at[0] - b.at[0]).pow(2) + (a.at[1] - b.at[1]).pow(2)) as f64;
            assert!(d.sqrt() > 30.0, "{a:?} {b:?}");
        }
    }
}

#[test]
fn the_same_body_gives_the_same_bytes() {
    let body = sphere(&Frame::WORLD, 5.0, 1).unwrap();
    let a = render(&body, &options(View::Iso)).unwrap().to_png();
    let b = render(&body, &options(View::Iso)).unwrap().to_png();
    assert_eq!(a, b);
    assert!(a.len() < 200_000, "{} bytes", a.len());
}

#[test]
fn four_views_fit_one_picture() {
    let body = box_solid(&Frame::WORLD, 10.0, 20.0, 30.0, 1).unwrap();
    let image = render_four(&body, &Options { width: 800, height: 600, ..Options::default() }).unwrap();
    assert_eq!((image.width, image.height), (800, 600));
    assert_eq!(image.rgb.len(), 800 * 600 * 3);
    assert_eq!(image.visible_faces.len(), 4, "top, front, right, and iso's three share those");
}

#[test]
fn awkward_sizes_do_not_panic() {
    let body = box_solid(&Frame::WORLD, 1.0, 1.0, 1000.0, 1).unwrap();
    for (w, h) in [(1, 1), (16, 4000), (5000, 16)] {
        let image = render(&body, &Options { width: w, height: h, ..Options::default() }).unwrap();
        assert!(!image.rgb.is_empty());
    }
    for view in [View::Top, View::Bottom, View::Orbit { azimuth: 400.0, elevation: -91.0 }] {
        render(&body, &options(view)).unwrap();
    }
}
