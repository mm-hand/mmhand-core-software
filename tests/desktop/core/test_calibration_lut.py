import pytest

from desktop.core.calibration_lut import CalibrationLutError, manual_encoder_to_urdf, resample_tendon_lut


def test_fixed_steps_preserve_true_final_urdf_endpoint():
    lut = resample_tendon_lut([(0, (0,)), (10, (100,)), (20, (200,))],
        urdf_min_degrees=0, urdf_max_degrees=17, actuator_count=1)
    assert lut.angles_degrees == (0, 5, 10, 15, 17)
    assert lut.motor_abs_rows == ((0, 50, 100, 150, 170),)


def test_dual_rows_keep_opposing_directions_and_extrapolate_end_slopes():
    lut = resample_tendon_lut([(20, (200, -200)), (10, (100, -100)), (0, (0, 0))],
        urdf_min_degrees=-5, urdf_max_degrees=25, actuator_count=2)
    assert lut.motor_abs_rows == ((-50, 0, 50, 100, 150, 200, 250),
                                  (50, 0, -50, -100, -150, -200, -250))


def test_rounding_is_symmetric_for_half_counts():
    lut = resample_tendon_lut([(0, (0, 0)), (10, (1, -1)), (20, (2, -2))],
        urdf_min_degrees=0, urdf_max_degrees=20, actuator_count=2)
    assert lut.motor_abs_rows[0][1] == 1
    assert lut.motor_abs_rows[1][1] == -1


def test_duplicate_angles_do_not_consume_lut_capacity():
    lut = resample_tendon_lut([(0, (0,)), (10, (90,)), (10, (100,)), (20, (200,))],
        urdf_min_degrees=0, urdf_max_degrees=20, actuator_count=1)
    assert lut.motor_abs_rows[0][2] == 100


@pytest.mark.parametrize("samples,maximum", [
    ([(0, (0,)), (5, (50,))], 20),
    ([(0, (0,)), (4, (40,)), (8, (80,))], 20),
    ([(0, (0,)), (10, (100,)), (20, (200,))], 117),
    ([(0, (0,)), (10, (float('nan'),)), (20, (200,))], 20),
])
def test_invalid_fit_is_rejected(samples, maximum):
    with pytest.raises(CalibrationLutError):
        resample_tendon_lut(samples, urdf_min_degrees=0,
                           urdf_max_degrees=maximum, actuator_count=1)


def test_manual_mapping_crosses_zero_and_preserves_outside_physical_scale():
    parameters = dict(manual_min_raw=16000, manual_max_raw=1000, direction=1,
                      urdf_min_degrees=10, urdf_max_degrees=60)
    assert manual_encoder_to_urdf(16000, **parameters) == 10
    assert manual_encoder_to_urdf(1000, **parameters) == 60
    assert manual_encoder_to_urdf(308, **parameters) == pytest.approx(35)
    assert manual_encoder_to_urdf(15999, **parameters) == pytest.approx(10 - 360 / 16384)
    assert manual_encoder_to_urdf(1001, **parameters) == pytest.approx(60 + 360 / 16384)


def test_manual_mapping_preserves_reverse_installation_direction():
    parameters = dict(manual_min_raw=1000, manual_max_raw=16000, direction=-1,
                      urdf_min_degrees=10, urdf_max_degrees=60)
    assert manual_encoder_to_urdf(308, **parameters) == pytest.approx(35)
    assert manual_encoder_to_urdf(1000, **parameters) == 10
    assert manual_encoder_to_urdf(16000, **parameters) == 60


@pytest.mark.parametrize("minimum,maximum", [(0, 0), (0, 8192), (-1, 100), (0, 16384)])
def test_manual_mapping_rejects_equal_reverse_or_invalid_raw(minimum, maximum):
    with pytest.raises(CalibrationLutError):
        manual_encoder_to_urdf(0, manual_min_raw=minimum, manual_max_raw=maximum,
            direction=1, urdf_min_degrees=0, urdf_max_degrees=90)
