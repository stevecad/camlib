use camlib::surface::SurfaceMesh;

fn mesh(error: f64) -> SurfaceMesh {
    SurfaceMesh {
        version: 1,
        units: "mm".into(),
        triangles: vec![[[0., 0., 1.], [1., 0., 1.], [0., 1., 1.]]].into(),
        surface_error_mm: error,
        source: serde_json::Value::Null,
    }
}

#[test]
fn surface_error_limits_are_inclusive_and_shared_with_floor_clipping() {
    for error in [0., 0.1, 0.1_f64.next_down()] {
        let mesh = mesh(error);
        assert!(mesh.prepare(0., 1e-5, &|| false).is_ok());
        assert!(mesh.above_floor(0., 1e-5, &|| false).is_ok());
    }
    for error in [
        -f64::EPSILON,
        0.1_f64.next_up(),
        0.117,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ] {
        let mesh = mesh(error);
        let message = mesh.prepare(0., 1e-5, &|| false).err().unwrap();
        assert!(message.contains("surface_error_mm"), "{message}");
        assert!(message.contains(&error.to_string()), "{message}");
        assert!(message.contains("0..=0.1 mm"), "{message}");
        assert_eq!(
            mesh.above_floor(0., 1e-5, &|| false).err().unwrap(),
            message
        );
    }
}

#[test]
fn surface_preparation_names_each_invalid_input() {
    for (field, value) in [
        ("version", "2"),
        ("units", "inch"),
        ("triangle count", "0"),
        ("base plane", "NaN"),
        ("tolerance", "0"),
    ] {
        let mut mesh = mesh(0.);
        let (mut floor, mut tolerance) = (0., 1e-5);
        match field {
            "version" => mesh.version = 2,
            "units" => mesh.units = "inch".into(),
            "triangle count" => mesh.triangles = vec![].into(),
            "base plane" => floor = f64::NAN,
            "tolerance" => tolerance = 0.,
            _ => unreachable!(),
        }
        let message = mesh.prepare(floor, tolerance, &|| false).err().unwrap();
        assert!(
            message.contains(field) && message.contains(value),
            "{message}"
        );
        assert_eq!(
            mesh.above_floor(floor, tolerance, &|| false).err().unwrap(),
            message
        );
    }
}
