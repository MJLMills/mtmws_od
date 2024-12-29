sleep_seconds = 0.01

lorenz_attractor_a = LoopableLorenzSystem()
lorenz_attractor_b = LoopableLorenzSystem()

computer = Computer()

led_matrix = computer.led_matrix
led_id_pulse_output_a = 1
led_id_pulse_output_b = 2
led_matrix.turn_on()

rate_knob = computer.main_knob
cv_magnitude_knob = computer.knob_x
divergence_knob = computer.knob_y
loop_switch = computer.switch_z

rate_cv_input = computer.cv_audio_input_socket_one
cv_magnitude_cv_input = computer.cv_audio_input_socket_two

cv_output_z_a = computer.cv_audio_output_socket_one
cv_output_x_a = computer.cv_output_socket_one

cv_output_z_b = computer.cv_audio_output_socket_two
cv_output_x_b = computer.cv_output_socket_two

led_pulse_output_length = 0.01

pulse_output_a = computer.pulses_output_socket_one
pulse_output_b = computer.pulses_output_socket_two

set_start_point = SetStartPoint(irq_source=computer.pulses_input_socket_one,
                                output=lorenz_attractor_a,
                                trigger=machine.Pin.IRQ_RISING)

toggle_looping = ToggleLooping(irq_source=computer.pulses_input_socket_two,
                               output=lorenz_attractor_a,
                               trigger=machine.Pin.IRQ_RISING)

time.sleep(0.5)  # pause after loading
led_matrix.turn_off()

rate_factor = 11 / 5

while True:

    cv_magnitude_value = 1.0 - cv_magnitude_knob.read_norm()
    rate_knob_value = 1.0 - rate_knob.read_norm()
    rate_cv_value = 1.0 - rate_cv_input.read_norm()

    # this writes to the LED state and pulse output so should be on the timer
    if lorenz_attractor_a.crossed_zero:
        led_matrix.turn_on(led_id_pulse_output_a)
        pulse_output_a.pulse(led_pulse_output_length)
        led_matrix.turn_off(led_id_pulse_output_a)

    if lorenz_attractor_b.crossed_zero:
        led_matrix.turn_on(led_id_pulse_output_b)
        pulse_output_b.pulse(led_pulse_output_length)
        led_matrix.turn_off(led_id_pulse_output_b)

    # the switch can be polled on a timer with the knobs and CV inputs

    # turn these back on once have figured out how to deal with the switch state
    # not matching the loop pulse status

    #    if loop_switch.is_up() and not lorenz_attractor_a.is_looping:
    #        lorenz_attractor_a.start_loop()
    #        lorenz_attractor_b.start_loop()

    #    elif loop_switch.is_middle() and lorenz_attractor_a.is_looping:
    #        lorenz_attractor_a.stop_loop()
    #        lorenz_attractor_b.stop_loop()

    #    elif loop_switch.is_down():
    #        lorenz_attractor_a.capture_initial_coordinates()
    #        lorenz_attractor_b.capture_initial_coordinates()

    if lorenz_attractor_a.is_looping:
        divergence_value = divergence_knob.read_norm()
        lorenz_attractor_a.random_factor = divergence_value

    # this is the CV output that should happen on a timer interrupt to be compute-independent
    # the values must be pre-computed to ints before the timer hits each time.

    cv_output_x_a.write_norm_value(
        cv_magnitude_value * lorenz_attractor_a.x_norm)
    cv_output_z_a.write(
        int(cv_magnitude_value * lorenz_attractor_a.z_norm * 65535))

    cv_output_x_b.write_norm_value(
        cv_magnitude_value * lorenz_attractor_b.x_norm)
    cv_output_z_b.write(
        int(cv_magnitude_value * lorenz_attractor_b.z_norm * 65535))

    time.sleep(rate_knob_value * rate_cv_value * sleep_seconds)

    lorenz_attractor_a.take_step(0.01)
    lorenz_attractor_b.take_step(0.01 / rate_factor)