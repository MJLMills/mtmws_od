import time
import LorenzSystem
import Computer

sleep_seconds = 0.01

lorenz_attractor_a = LorenzSystem()
lorenz_attractor_b = LorenzSystem()

main_knob = Computer.MainKnob()

cv_output_z_a = Computer.CVAudioOutputSocketOne()
cv_output_x_a = Computer.CVOutputSocketOne()

cv_output_z_b = Computer.CVAudioOutputSocketTwo()
cv_output_x_b = Computer.CVOutputSocketTwo()

led_matrix = Computer.LEDMatrix()
led_id_pulse_output_a = 1
led_id_pulse_output_b = 2
led_pulse_output_length = 0.01

pulse_output_a = Computer.PulseOutputSocketOne()
pulse_output_b = Computer.PulseOutputSocketTwo()

while True:

    lorenz_attractor_a.take_step()
    lorenz_attractor_b.take_step()

    cv_output_x_a.write(int(lorenz_attractor_a.x_norm * 65535))
    cv_output_z_a.write(int(lorenz_attractor_a.z_norm * 65535))

    cv_output_x_b.write_norm_value(lorenz_attractor_b.x_norm)
    cv_output_z_b.write_norm_value(lorenz_attractor_b.z_norm)

    if lorenz_attractor_a.crossed_zero:
        led_matrix.turn_on(led_id_pulse_output_a)
        pulse_output_a.pulse(led_pulse_output_length)
    else:
        led_matrix.turn_off(led_id_pulse_output_a)

    lorenz_attractor_b.take_step()
    if lorenz_attractor_b.crossed_zero:
        led_matrix.turn_on(led_id_pulse_output_b)
        pulse_output_b.pulse(led_pulse_output_length)
    else:
        led_matrix.turn_off(led_id_pulse_output_b)

    main_knob_value = main_knob.read_norm()

    time.sleep(main_knob_value * sleep_seconds)

