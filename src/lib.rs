type AngleArray = [f32; 9];
static ANGLES: AngleArray = [-180., -135., -90., -45., 0., 45., 90., 135., 180.];
static MOTOR_A_ANGLE_SPEEDS: AngleArray = [-1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0];
static MOTOR_B_ANGLE_SPEEDS: AngleArray = [-1.0, 0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0];

pub fn lerp(value: f32, min_input: f32, max_input: f32, min_output: f32, max_output: f32) -> f32 {
    return min_output
        + (value.clamp(min_input, max_input) - min_input) / (max_input - min_input)
            * (max_output - min_output);
}

fn lerp_array(value: f32, inputs: AngleArray, outputs: AngleArray) -> f32 {
    for (idx, &cur) in inputs[0..inputs.len() - 1].into_iter().enumerate() {
        let next = inputs[idx + 1];
        if value < cur {
            return outputs[idx];
        } else if value <= next {
            return lerp(value, cur, next, outputs[idx], outputs[idx + 1]);
        }
    }
    outputs[outputs.len() - 1]
}

// Calcula el vector de direccion e intensidad para los motores de lados A y B
// basados en las coordenadas x, y del joystick.
#[inline]
pub fn calculate_motors_direction_velocity_vector(x: f32, y: f32) -> (f32, f32) {
    let rad_angle = x.atan2(y);
    let deg_angle = rad_angle * (180.0 / std::f32::consts::PI);

    // Mangitud al cuadrado. Esto nos da una magnitud de 1.3 en las diagonales y
    // 1 en los cardinales.
    let magnitude = x * x + y * y;

    // // Mangitud directa. Esto nos da una magnitud de 1.15 en las diagonales y
    // // 1 en los cardinales.
    // let squared_magnitude = x * x + y * y;
    // let magnitude = squared_magnitude.sqrt();

    // // Este mecanismo para obtener la magnitud nos deja con una magnitud de 0.8
    // // en las diagonales y 1 en los cardinales.
    // // https://answers.unity.com/questions/804075/how-to-get-overall-intensity-for-each-joystick-axi.html
    // let max_magnitude = if x.abs() > y.abs() { 1. / rad_angle.sin() } else { 1. / rad_angle.cos() };
    // let squared_magnitude = x * x + y * y;
    // let magnitude = (squared_magnitude.sqrt() / (max_magnitude)).abs();

    let magnitude = lerp(magnitude, 0.1, 1.0, 0., 1.);
    let m1v = lerp_array(deg_angle, ANGLES, MOTOR_A_ANGLE_SPEEDS) * magnitude;
    let m2v = lerp_array(deg_angle, ANGLES, MOTOR_B_ANGLE_SPEEDS) * magnitude;

    // println!(
    //     "x={:.3} \ty={:.3} \trad_angle={:.3} \tdeg_angle={:.3} \tmangitude={:.3} \tm1_magnitude={:.3} \tm2_magnitude={:.3}",
    //     x, y, rad_angle, deg_angle, magnitude, m1_magnitude, m2_magnitude
    // );

    (m1v, m2v)
}

#[cfg(test)]
mod tests {
    use crate::*;

    fn test_motor_vectors(x: f32, y: f32, m1v_expected: f32, m2v_expected: f32) {
        let (m1v, m2v) = calculate_motors_direction_velocity_vector(x, y);
        assert_eq!(m1v, m1v_expected);
        assert_eq!(m2v, m2v_expected);
    }

    #[test]
    fn test_lerp_01() {
        assert_eq!(lerp(0.0, 0., 1., 0., 100.), 0.)
    }
    #[test]
    fn test_lerp_02() {
        assert_eq!(lerp(0.5, 0., 1., 0., 100.), 50.)
    }
    #[test]
    fn test_lerp_03() {
        assert_eq!(lerp(1.0, 0., 1., 0., 100.), 100.)
    }
    #[test]
    fn test_lerp_04() {
        assert_eq!(lerp(75.0, 50., 100., 0., 100.), 50.)
    }
    #[test]
    fn test_lerp_min_clamp() {
        assert_eq!(lerp(1.0, 50., 100., 0., 100.), 0.)
    }
    #[test]
    fn test_lerp_max_clamp() {
        assert_eq!(lerp(110.0, 50., 100., 0., 100.), 100.)
    }

    #[test]
    fn test_motor_vectors_stopped() {
        test_motor_vectors(0., 0., 0., 0.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_n() {
        test_motor_vectors(0., 1., 1., 1.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_nw() {
        test_motor_vectors(-1., 1., 0., 1.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_e() {
        test_motor_vectors(-1., 0., -1., 1.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_se() {
        test_motor_vectors(-1., -1., -1., 0.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_s() {
        test_motor_vectors(0., -1., -1., -1.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_sw() {
        test_motor_vectors(-1., -1., -1., 0.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_w() {
        test_motor_vectors(1., 0., 1., -1.);
    }

    #[test]
    fn test_motor_vectors_full_throttle_ne() {
        test_motor_vectors(1., 1., 1., -0.);
    }

    #[test]
    fn test_lerp_motor_exact() {
        ANGLES.iter().enumerate().for_each(|(idx, &i)| {
            assert_eq!(lerp_array(i, ANGLES, MOTOR_A_ANGLE_SPEEDS), MOTOR_A_ANGLE_SPEEDS[idx]);
            assert_eq!(lerp_array(i, ANGLES, MOTOR_B_ANGLE_SPEEDS), MOTOR_B_ANGLE_SPEEDS[idx]);
        });
    }
    #[test]
    fn test_lerp_motor_interpolated() {
        assert_eq!(lerp_array(-67.5, ANGLES, MOTOR_A_ANGLE_SPEEDS), -0.5);
    }
    #[test]
    fn test_lerp_motor_min_clamp() {
        assert_eq!(lerp_array(-1000., ANGLES, MOTOR_A_ANGLE_SPEEDS), MOTOR_A_ANGLE_SPEEDS[0]);
    }
    #[test]
    fn test_lerp_motor_max_clamp() {
        assert_eq!(lerp_array(1000., ANGLES, MOTOR_A_ANGLE_SPEEDS), MOTOR_A_ANGLE_SPEEDS[MOTOR_A_ANGLE_SPEEDS.len() - 1]);
    }

}
