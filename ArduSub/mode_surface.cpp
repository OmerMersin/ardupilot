#include "Sub.h"

bool ModeSurface::init(bool ignore_checks)
{
    // vertical controller limits
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(),
                                               g.pilot_speed_up,
                                               g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cm(-sub.get_pilot_speed_dn(),
                                                      g.pilot_speed_up,
                                                      g.pilot_accel_z);

    // init controller
    position_control->init_U_controller();

    // Latch initial targets
    _z0_cm = sub.inertial_nav.get_position_z_up_cm();

#if AP_RANGEFINDER_ENABLED
    if (sub.rangefinder_alt_ok()) {
        _rng_target_cm = sub.rangefinder_state.alt * 100.0f; // meters -> cm
        _target_set = true;
        gcs().send_text(MAV_SEVERITY_INFO, "SURFACE: RF hold %.1f cm", (double)_rng_target_cm);
    } else {
        _target_set = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "SURFACE: RF not healthy, holding current Z");
    }
#else
    _target_set = false;
    gcs().send_text(MAV_SEVERITY_WARNING, "SURFACE: Rangefinder disabled");
#endif

    // Start by holding current EKF Z
    position_control->set_pos_desired_U_cm(_z0_cm);
    return true;
}

void ModeSurface::run()
{
    // if not armed, idle outputs and reset controller
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->init_U_controller();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_RANGEFINDER_ENABLED
    if (sub.rangefinder_alt_ok()) {
        // If we didn't have a target yet (became healthy after entry), latch it now
        if (!_target_set) {
            _rng_target_cm = sub.rangefinder_state.alt * 100.0f; // meters -> cm
            _z0_cm = sub.inertial_nav.get_position_z_up_cm();
            _target_set = true;
            gcs().send_text(MAV_SEVERITY_INFO, "SURFACE: RF hold %.1f cm", (double)_rng_target_cm);
        }

        // Map RF error to EKF-Z target: desired_Z = Z0 - (RF - RF0)
        const float current_rf_cm = sub.rangefinder_state.alt * 100.0f;
        const float desired_z_cm = _z0_cm - (current_rf_cm - _rng_target_cm);

        position_control->set_pos_desired_U_cm(desired_z_cm);
    }
    // else: keep last desired Z if RF unhealthy (no baro/depth fallback)
#endif

    // Command zero climb rate and run vertical controller
    position_control->set_pos_target_U_from_climb_rate_cm(0.0f);
    position_control->update_U_controller();

    // Pilot horizontal translation (no attitude stabilization requested)
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
