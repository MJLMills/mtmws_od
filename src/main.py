INPUT_SOCKET_UPDATE_STEPS = 32
"""The number of iterations of the main loop before the input socket states are checked."""

MINIMUM_TIMESTEP: float = 0.0001
"""min timestep has no mathematical limit, must be arbitrarily chosen."""

MAXIMUM_TIMESTEP: float = 0.01
"""max timestep is limited by the Euler method integration accuracy."""

TIMESTEP_FACTOR: float = 5 / 11
"""sets how much slower B is than A."""

DIVERGENCE_MIN: float = 0.0
"""min knob/CV settings will turn off sensitivity to initial conditions."""

DIVERGENCE_MAX: float = 2.0
"""max divergence has no mathematical limit, must be arbitrarily chosen."""

timestep_a = RangedVariable(
    minimum=MINIMUM_TIMESTEP,
    maximum=MAXIMUM_TIMESTEP,
    value=MAXIMUM_TIMESTEP
)

timestep_b = RangedVariable(
    minimum=MINIMUM_TIMESTEP * TIMESTEP_FACTOR,
    maximum=MAXIMUM_TIMESTEP * TIMESTEP_FACTOR,
    value=MAXIMUM_TIMESTEP * TIMESTEP_FACTOR
)

divergence = RangedVariable(minimum=DIVERGENCE_MIN,
                            maximum=DIVERGENCE_MAX,
                            value=DIVERGENCE_MIN)

lorenz_attractor_a = LorenzSystem(
    timestep=timestep_a,
    random_factor=divergence
)
"""The Lorenz attractor running at the faster rate."""

lorenz_attractor_b = LorenzSystem(
    timestep=timestep_b,
    random_factor=divergence
)
"""The Lorenz attractor running at the slower rate."""

looper = Looper(models=[lorenz_attractor_a, lorenz_attractor_b])
"""The looper that controls the time-stepping of multiple models."""

computer = Computer()
computer.led_matrix.turn_off()
main_knob = computer.main_knob
knob_x = computer.knob_x
knob_y = computer.knob_y
switch_z = computer.switch_z

# lorenz A

crossed_zero_led_a = computer.led_matrix.get_by_index(1)
crossed_zero_pulse_output_socket_a = computer.pulses_output_socket_one

lorenz_attractor_a_x_cv_output_socket = computer.cv_output_socket_one
lorenz_attractor_a_z_cv_output_socket = computer.cv_audio_output_socket_one

lorenz_attractor_a.crossed_zero.connect(crossed_zero_led_a.pulse)
lorenz_attractor_a.crossed_zero.connect(
    crossed_zero_pulse_output_socket_a.pulse)
lorenz_attractor_a.x_changed.connect(
    lorenz_attractor_a_x_cv_output_socket.map_and_write_value)
lorenz_attractor_a.z_changed.connect(
    lorenz_attractor_a_z_cv_output_socket.map_and_write_value)

main_knob.value_changed.connect(lorenz_attractor_a.timestep.map_value)
knob_y.value_changed.connect(lorenz_attractor_a.random_factor.map_value)

# lorenz B

crossed_zero_led_b = computer.led_matrix.get_by_index(2)
crossed_zero_pulse_output_socket_b = computer.pulses_output_socket_two

lorenz_attractor_b_x_cv_output_socket = computer.cv_output_socket_two
lorenz_attractor_b_z_cv_output_socket = computer.cv_audio_output_socket_two

lorenz_attractor_b.crossed_zero.connect(crossed_zero_led_b.pulse)
lorenz_attractor_b.crossed_zero.connect(
    crossed_zero_pulse_output_socket_b.pulse)
lorenz_attractor_b.x_changed.connect(
    lorenz_attractor_b_x_cv_output_socket.map_and_write_value)
lorenz_attractor_b.z_changed.connect(
    lorenz_attractor_b_z_cv_output_socket.map_and_write_value)

main_knob.value_changed.connect(lorenz_attractor_b.timestep.map_value)
knob_y.value_changed.connect(lorenz_attractor_b.random_factor.map_value)

# hardware
# Knob X acts as a VCA on all four output CVs when CV/Audio input sockets are unplugged
# CV/Audio inputs act as VCAs on A and B output CVs

# set the input range (min, max of the ranged variable) on the socket in V or values
cv_magnitude_a_input_socket = computer.cv_audio_input_socket_one
# cv_magnitude_a_input_socket.jack_inserted.connect(printit)

# assume jack is disconnected
knob_x.value_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_range)

# if a jack is plugged into the CV_a magnitude input socket,
cv_magnitude_a_input_socket.jack_inserted.connect(
    lambda: print("CV/Audio A (one) jack inserted"),
    # disconnect knob_x from setting the x,z output socket ranges
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
    # and connect the socket to set the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.connect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.connect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
)

# if a jack is removed from the CV_a magnitude input socket,
cv_magnitude_a_input_socket.jack_removed.connect(
    lambda: print("CV/Audio A (one) jack removed"),
    # connect knob_x to set the x,z output socket ranges
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
    # and disconnect the socket from setting the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(
        lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(
        lorenz_attractor_a_z_cv_output_socket.map_range),
    # update the range of the CV output to match the knob value
    lambda: lorenz_attractor_a_x_cv_output_socket.map_range(
        knob_x.ranged_variable)
)

cv_magnitude_b_input_socket = computer.cv_audio_input_socket_two

knob_x.value_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_range)

# if a jack is plugged into the CV_b magnitude input socket,
cv_magnitude_b_input_socket.jack_inserted.connect(
    lambda: print("CV/Audio B (two) jack inserted"),
    # disconnect knob_x from setting the x,z output socket ranges
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.disconnect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
    # and connect the socket to set the x, z output ranges
    lambda: cv_magnitude_b_input_socket.value_changed.connect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_b_input_socket.value_changed.connect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
)

# if a jack is removed from the CV_b magnitude input socket,
cv_magnitude_b_input_socket.jack_removed.connect(
    lambda: print("CV/Audio B (two) jack removed"),
    # connect knob_x to set the x,z output socket ranges
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: knob_x.value_changed.connect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
    # and disconnect the socket from setting the x, z output ranges
    lambda: cv_magnitude_b_input_socket.value_changed.disconnect(
        lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_b_input_socket.value_changed.disconnect(
        lorenz_attractor_b_z_cv_output_socket.map_range),
    # update the range of the CV output to match the knob value
    lambda: lorenz_attractor_b_x_cv_output_socket.map_range(
        knob_x.ranged_variable)
)

timestep_cv_input_socket = computer.cv_input_socket_one
# if a jack is plugged into the timestep input socket,
timestep_cv_input_socket.jack_inserted.connect(
    lambda: print("Timestep (one) jack inserted"),
    # disconnect the timestep knob from the timesteps of A and B
    lambda: main_knob.value_changed.disconnect(
        lorenz_attractor_a.timestep.map_value),
    lambda: main_knob.value_changed.disconnect(
        lorenz_attractor_b.timestep.map_value),
    # connect the timestep input socket to the timesteps of A and B
    lambda: timestep_cv_input_socket.value_changed.connect(
        lorenz_attractor_a.timestep.map_value),
    lambda: timestep_cv_input_socket.value_changed.connect(
        lorenz_attractor_b.timestep.map_value),
)

# if a jack is removed from the timestep input socket,
timestep_cv_input_socket.jack_removed.connect(
    lambda: print("Timestep (one) jack removed"),
    # connect the timestep knob to the timesteps of A and B
    lambda: main_knob.value_changed.connect(
        lorenz_attractor_a.timestep.map_value),
    lambda: main_knob.value_changed.connect(
        lorenz_attractor_b.timestep.map_value),
    # disconnect the timestep input socket from the timesteps of A and B
    lambda: timestep_cv_input_socket.value_changed.disconnect(
        lorenz_attractor_a.timestep.map_value),
    lambda: timestep_cv_input_socket.value_changed.disconnect(
        lorenz_attractor_b.timestep.map_value),
)

divergence_cv_input_socket = computer.cv_input_socket_two
# divergence_cv_input_socket.jack_inserted.connect(printit)
divergence_cv_input_socket.jack_inserted.connect(
    lambda: print("Divergence (CV two) jack inserted"),
    # disconnect the divergence knob from the divergence of A and B
    knob_y.value_changed.disconnect(
        lorenz_attractor_a.random_factor.map_value),
    knob_y.value_changed.disconnect(
        lorenz_attractor_b.random_factor.map_value),
    # connect the divergence socket to the divergence of A and B
    divergence_cv_input_socket.value_changed.connect(
        lorenz_attractor_a.random_factor.map_value),
    divergence_cv_input_socket.value_changed.connect(
        lorenz_attractor_b.random_factor.map_value),
)

divergence_cv_input_socket.jack_removed.connect(
    lambda: print("Divergence (CV two) jack removed"),
    # connect the divergence knob to the divergence of A and B
    knob_y.value_changed.connect(lorenz_attractor_a.random_factor.map_value),
    knob_y.value_changed.connect(lorenz_attractor_b.random_factor.map_value),
    # disconnect the divergence socket from the divergence of A and B
    divergence_cv_input_socket.value_changed.disconnect(
        lorenz_attractor_a.random_factor.map_value),
    divergence_cv_input_socket.value_changed.disconnect(
        lorenz_attractor_b.random_factor.map_value),
)

# looper

looper_status_led = computer.led_matrix.get_by_index(3)
set_start_led = computer.led_matrix.get_by_index(4)
loop_begin_led_one = computer.led_matrix.get_by_index(5)
loop_begin_led_two = computer.led_matrix.get_by_index(6)
switch_z = computer.switch_z
loop_toggle_pulse_socket = computer.pulses_input_socket_one
loop_set_initial_coordinates_socket = computer.pulses_input_socket_two

# connect the looper's signals to slots (all LED-controlling functions)
looper.looping_started.connect(looper_status_led.turn_on)
looper.at_loop_start.connect(loop_begin_led_one.pulse)
looper.at_loop_start.connect(loop_begin_led_two.pulse)
looper.looping_stopped.connect(looper_status_led.turn_off)
looper.initial_coordinates_set.connect(set_start_led.pulse)

# connect switch Z signals to slots
if switch_z.is_up():
    looper.start_looping()

# if switch Z is moved to the up position,
switch_z.switched_up.connect(
    # start looping the models
    looper.start_looping,
    # and disconnect the pulse input toggling looping
    lambda: loop_toggle_pulse_socket.pulse_started.disconnect(
        looper.toggle_looping)
)

# if switch Z is moved from the up to the middle position,
switch_z.switched_up_to_middle.connect(
    # stop looping the models
    looper.stop_looping,
    # and if the pulse input toggling looping has a jack in it, connect it to toggle looping
    lambda: loop_toggle_pulse_socket.has_jack and loop_toggle_pulse_socket.pulse_started.connect(
        looper.toggle_looping)
)

# if switch Z is moved to the down position, set the initial coordinates of the looper
switch_z.switched_down.connect(looper.set_initial_coordinates)

# if a jack is inserted into the loop toggle pulse input,
loop_toggle_pulse_socket.jack_inserted.connect(
    # connect the socket to toggle looping if the switch is in the middle
    lambda: switch_z.is_middle and loop_toggle_pulse_socket.pulse_started.connect(
        looper.toggle_looping)
)

# if a jack is removed from the loop toggle pulse input,
loop_toggle_pulse_socket.jack_removed.connect(
    # disconnect the socket to prevent toggling of looping
    lambda: loop_toggle_pulse_socket.pulse_started.disconnect(
        looper.toggle_looping)
)

# if a jack is inserted into the set initial coordinates pulse input,
loop_set_initial_coordinates_socket.jack_inserted.connect(
    # connect the socket to set the initial coordinates
    lambda: loop_set_initial_coordinates_socket.pulse_started.connect(
        looper.set_initial_coordinates)
)

# if a jack is removed from the loop toggle pulse input,
loop_set_initial_coordinates_socket.jack_removed.connect(
    # disconnect the socket to prevent setting the initial coordinates
    lambda: loop_set_initial_coordinates_socket.pulse_started.disconnect(
        looper.set_initial_coordinates)
)

write_output = TimerConnector(
    looper=looper,
    freq=275,  # 275 max
    computer=computer,
)

computer.update_input_sockets()
counter = 0
while True:

    if counter == INPUT_SOCKET_UPDATE_STEPS:
        computer.update_input_sockets()
        counter = 0
    counter += 1

    computer.read_analog_inputs()