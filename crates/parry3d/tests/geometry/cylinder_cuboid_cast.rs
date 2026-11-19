use parry3d::math::{Pose, Vector};
use parry3d::query::{self, ShapeCastOptions};
use parry3d::shape::{Ball, Capsule, Cuboid, Cylinder, Shape};

/// A cylinder standing upright and swept level over a box's top edge, its flat bottom a few
/// millimetres to a few centimetres above the box's top, never touches it. The GJK ray cast
/// stalled on the two parallel flat faces and returned its lower bound as a hit, with a made-up
/// normal: a third of these casts hit, at clearances up to 14.5 cm (a character's capsule-less
/// controller could not step up a riser its step was meant to take).
#[test]
fn cylinder_swept_over_a_box_edge_does_not_hit_it() {
    let cylinder = Cylinder::new(0.8685, 0.24);
    let step = Cuboid::new(Vector::new(10.0, 0.15, 2.0));
    let step_at = Pose::from_translation(Vector::new(0.0, 0.15, -7.0));
    let mut hits = vec![];
    for xi in 0..20 {
        for ci in 1..240 {
            let clearance = ci as f32 * 0.00125;
            for zi in 0..10 {
                let at = Pose::from_translation(Vector::new(
                    xi as f32 * 0.0007,
                    0.30 + clearance + 0.8685,
                    -4.70 - zi as f32 * 0.006,
                ));
                let hit = query::cast_shapes(
                    &at,
                    Vector::new(0.0, 0.0, -1.0),
                    &cylinder,
                    &step_at,
                    Vector::ZERO,
                    &step,
                    ShapeCastOptions::with_max_time_of_impact(0.165),
                )
                .unwrap();
                if hit.is_some() {
                    hits.push(clearance);
                }
            }
        }
    }
    assert!(
        hits.is_empty(),
        "{} casts over the edge hit it, at clearances up to {:?} m",
        hits.len(),
        hits.iter().cloned().reduce(f32::max)
    );
}

/// The same cylinder swept with its bottom below the box's top hits the box's face where it
/// meets it.
#[test]
fn cylinder_swept_into_a_box_face_hits_it_there() {
    let cylinder = Cylinder::new(0.8685, 0.24);
    let step = Cuboid::new(Vector::new(10.0, 0.15, 2.0));
    let step_at = Pose::from_translation(Vector::new(0.0, 0.15, -7.0));
    for below in [0.01f32, 0.05, 0.15, 0.29] {
        let at = Pose::from_translation(Vector::new(0.0, 0.30 - below + 0.8685, -4.0));
        let hit = query::cast_shapes(
            &at,
            Vector::new(0.0, 0.0, -1.0),
            &cylinder,
            &step_at,
            Vector::ZERO,
            &step,
            ShapeCastOptions::with_max_time_of_impact(2.0),
        )
        .unwrap()
        .unwrap_or_else(|| panic!("{below} m below the top: missed the face"));
        // The cylinder's front, 0.24 ahead of its axis, meets the face 1 m on at z -5.
        assert!(
            (hit.time_of_impact - 0.76).abs() < 2.0e-3,
            "{below} m below the top: hit at {}",
            hit.time_of_impact
        );
    }
}

/// Casts from random places in random directions at a box: every hit is where the shapes touch,
/// with nothing touched before it, and every miss never touches.
#[test]
fn casts_at_a_box_hit_where_they_touch_and_miss_where_they_do_not() {
    let step = Cuboid::new(Vector::new(10.0, 0.15, 2.0));
    let step_at = Pose::from_translation(Vector::new(0.0, 0.15, -7.0));
    let shapes: [(&str, Box<dyn Shape>); 4] = [
        ("cylinder", Box::new(Cylinder::new(0.8685, 0.24))),
        ("capsule", Box::new(Capsule::new_y(0.6, 0.24))),
        ("cuboid", Box::new(Cuboid::new(Vector::new(0.3, 0.5, 0.2)))),
        ("ball", Box::new(Ball::new(0.05))),
    ];
    let mut seed = 12345u64;
    let mut random = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f32 / (1u64 << 53) as f32
    };
    for (name, shape) in &shapes {
        for _ in 0..3000 {
            let from = Vector::new(
                random() * 4.0 - 2.0,
                random() * 2.0 - 0.5,
                -8.5 + random() * 8.0,
            );
            let mut dir = Vector::new(random() - 0.5, (random() - 0.5) * 0.6, random() - 0.5);
            if random() < 0.5 {
                dir.y = 0.0;
            }
            let dir = dir.normalize();
            let distance_at = |t: f32| {
                let at = Pose::from_translation(from + dir * t);
                query::contact(&at, shape.as_ref(), &step_at, &step, 10.0)
                    .unwrap()
                    .map_or(10.0, |c| c.dist)
            };
            if distance_at(0.0) <= 0.0 {
                continue;
            }
            let longest = 3.0;
            match query::cast_shapes(
                &Pose::from_translation(from),
                dir,
                shape.as_ref(),
                &step_at,
                Vector::ZERO,
                &step,
                ShapeCastOptions::with_max_time_of_impact(longest),
            )
            .unwrap()
            {
                Some(hit) => {
                    let t = hit.time_of_impact;
                    assert!(
                        distance_at(t) <= 5.0e-3,
                        "{name} from {from} along {dir}: hit at {t} {} apart",
                        distance_at(t)
                    );
                    for i in 0..50 {
                        let s = t * i as f32 / 50.0;
                        assert!(
                            distance_at(s) > -1.0e-3,
                            "{name} from {from} along {dir}: hit at {t}, touched at {s}"
                        );
                    }
                }
                None => {
                    for i in 0..=150 {
                        let s = longest * i as f32 / 150.0;
                        assert!(
                            distance_at(s) > -1.0e-3,
                            "{name} from {from} along {dir}: missed, touched at {s}"
                        );
                    }
                }
            }
        }
    }
}
