# Define constants

MAXIMUM_TIMER_FREQUENCY_HZ = 180
"""The maximum frequency (in Hz) at which the timed loop will be executed.
The upper bound of this value is limited by the execution time of the timed
loop, which is determined by the model update and output socket write timings.
The program will run at this frequency, so in combination with the upper limit
on the model system timestep (limited by integration accuracy) this sets the
fastest speed at which the program can run.
"""

INPUT_SOCKET_UPDATE_STEPS = 24
"""The iteration count of the main loop before the input sockets are checked.
This should be set low enough that the program detects the plugging and 
unplugging of jacks in the input socket effectively instantly, but high enough
that cycles are not wasted checking the normalization probe.
"""

MINIMUM_TIMESTEP: float = 0.0001
"""The slowest settable timestep of the Lorenz system.
This value has no mathematical lower limit, so it is arbitrarily chosen.
When a jack is plugged into the timestep CV input, the minimum timestep
is controlled by the timestep knob between the defined maximum timestep
and a value a factor max_timestep/min_timestep smaller than this value.
"""

MAXIMUM_TIMESTEP: float = 0.01
"""The fastest settable timestep of the Lorenz system.
The upper limit of this value depends on the Euler method integration accuracy.
"""

TIMESTEP_FACTOR: float = 5 / 11
"""Determines how much slower Lorenz attractor B is than Lorenz attractor A.
This factor is multiplicative so must be between 0 and 1.
"""

RANDOM_FACTOR_MIN: float = 0.0
"""The smallest random factor to apply to the position on trajectory reset.
A value of zero allows turning off this sensitivity with hardware controls.
"""

RANDOM_FACTOR_MAX: float = 2.0
"""The largest random factor to apply to the position on trajectory reset.
This value has no mathematical limit and is arbitrarily chosen.
Too low a value makes the sensitivity audibly difficult to detect.
Too high a value makes the looping audibly difficult to detect.
When a jack is plugged into the random factor CV input, the maximum
random factor is controlled by the random factor knob.
"""

# Create the software ranged variables.

timestep_a = RangedVariable(
    minimum=RangedVariable(minimum=MINIMUM_TIMESTEP / (MAXIMUM_TIMESTEP / MINIMUM_TIMESTEP),
                           maximum=MAXIMUM_TIMESTEP,
                           value=MINIMUM_TIMESTEP),
    maximum=MAXIMUM_TIMESTEP,
    value=MAXIMUM_TIMESTEP
)
"""The timestep ranged variable for Lorenz attractor A."""

b_minimum_timestep = MINIMUM_TIMESTEP * TIMESTEP_FACTOR
b_maximum_timestep = MAXIMUM_TIMESTEP * TIMESTEP_FACTOR

timestep_b = RangedVariable(
    minimum=RangedVariable(minimum=b_minimum_timestep / (b_maximum_timestep / b_minimum_timestep),
                           maximum=b_maximum_timestep,
                           value=b_minimum_timestep),
    maximum=b_maximum_timestep,
    value=b_maximum_timestep
)
"""The timestep ranged variable for Lorenz attractor B."""

random_factor = RangedVariable(
    minimum=RANDOM_FACTOR_MIN,
    maximum=RangedVariable(minimum=RANDOM_FACTOR_MAX - 1,
                           maximum=RANDOM_FACTOR_MAX + 1,
                           value=RANDOM_FACTOR_MAX),
    value=RANDOM_FACTOR_MIN
)
"""The random factor ranged variable for both Lorenz attractors."""

# Create the model variables.

lorenz_attractor_a = LorenzSystem(
    timestep=timestep_a,
    random_factor=random_factor
)
"""The Lorenz attractor running at the faster rate (A)."""

lorenz_attractor_b = LorenzSystem(
    timestep=timestep_b,
    random_factor=random_factor
)
"""The Lorenz attractor running at the slower rate (B)."""

looper = Looper(models=[lorenz_attractor_a, lorenz_attractor_b])
"""The looper that controls the time-stepping of multiple models."""

# Create the computer hardware variables and make signal-slot connections.

computer = Computer()
computer.led_matrix.turn_on()  # signal the user that setup has started
timestep_knob = computer.main_knob
cv_magnitude_knob = computer.knob_x
random_factor_knob = computer.knob_y
switch_z = computer.switch_z

# lorenz A
# The LED, pulse output and CV output are always connected to the model.

crossed_zero_led_a = computer.led_matrix.get_by_index(1)
crossed_zero_pulse_output_socket_a = computer.pulses_output_socket_one

lorenz_attractor_a_x_cv_output_socket = computer.cv_output_socket_one
lorenz_attractor_a_z_cv_output_socket = computer.cv_audio_output_socket_one

lorenz_attractor_a.crossed_zero.connect(crossed_zero_led_a.pulse)
lorenz_attractor_a.crossed_zero.connect(crossed_zero_pulse_output_socket_a.pulse)
lorenz_attractor_a.x_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_and_write_value)
lorenz_attractor_a.z_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_and_write_value)

# lorenz B
# The LED, pulse output and CV output are always connected to the model.

crossed_zero_led_b = computer.led_matrix.get_by_index(2)
crossed_zero_pulse_output_socket_b = computer.pulses_output_socket_two

lorenz_attractor_b_x_cv_output_socket = computer.cv_output_socket_two
lorenz_attractor_b_z_cv_output_socket = computer.cv_audio_output_socket_two

lorenz_attractor_b.crossed_zero.connect(crossed_zero_led_b.pulse)
lorenz_attractor_b.crossed_zero.connect(crossed_zero_pulse_output_socket_b.pulse)
lorenz_attractor_b.x_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_and_write_value)
lorenz_attractor_b.z_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_and_write_value)

# Knob X acts as a VCA on all four output CVs when CV/Audio input sockets are unplugged
# CV/Audio inputs act as VCAs on A and B output CVs when they are plugged

# CV input socket one (amplitude of CV outputs Ax and Az
cv_magnitude_a_input_socket = computer.cv_audio_input_socket_one
cv_magnitude_a_input_socket.set_voltage_range((0, 5))

# if a jack is plugged into the CV_a magnitude input socket,
cv_magnitude_a_input_socket.jack_inserted.connect(
    #lambda: print("CV/Audio A (one) jack inserted"),
    # disconnect knob_x from setting the x,z output socket ranges
    lambda: cv_magnitude_knob.value_changed.disconnect(lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_knob.value_changed.disconnect(lorenz_attractor_a_z_cv_output_socket.map_range),
    # and connect the socket to set the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_range),
    # update the range of the CV output to match the socket value
    lambda: lorenz_attractor_a_x_cv_output_socket.map_range(cv_magnitude_a_input_socket.ranged_variable),
    lambda: lorenz_attractor_a_z_cv_output_socket.map_range(cv_magnitude_a_input_socket.ranged_variable),
)

# if a jack is removed from the CV_a magnitude input socket,
cv_magnitude_a_input_socket.jack_removed.connect(
    #lambda: print("CV/Audio A (one) jack removed"),
    # connect knob_x to set the x,z output socket ranges
    lambda: cv_magnitude_knob.value_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_knob.value_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_range),
    # and disconnect the socket from setting the x, z output ranges
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(lorenz_attractor_a_x_cv_output_socket.map_range),
    lambda: cv_magnitude_a_input_socket.value_changed.disconnect(lorenz_attractor_a_z_cv_output_socket.map_range),
    # update the range of the CV output to match the knob value
    lambda: lorenz_attractor_a_x_cv_output_socket.map_range(cv_magnitude_knob.ranged_variable),
    lambda: lorenz_attractor_a_z_cv_output_socket.map_range(cv_magnitude_knob.ranged_variable),
)

cv_magnitude_b_input_socket = computer.cv_audio_input_socket_two
cv_magnitude_b_input_socket.set_voltage_range((0, 5))

# if a jack is plugged into the CV_b magnitude input socket,
cv_magnitude_b_input_socket.jack_inserted.connect(
    #lambda: print("CV/Audio B (two) jack inserted"),
    # disconnect knob_x from setting the x,z output socket ranges
    lambda: cv_magnitude_knob.value_changed.disconnect(lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_knob.value_changed.disconnect(lorenz_attractor_b_z_cv_output_socket.map_range),
    # and connect the socket to set the x, z output ranges
    lambda: cv_magnitude_b_input_socket.value_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_b_input_socket.value_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_range),
    # update the range of the CV output to match the socket value
    lambda: lorenz_attractor_b_x_cv_output_socket.map_range(cv_magnitude_b_input_socket.ranged_variable),
    lambda: lorenz_attractor_b_z_cv_output_socket.map_range(cv_magnitude_b_input_socket.ranged_variable),
)

# if a jack is removed from the CV_b magnitude input socket,
cv_magnitude_b_input_socket.jack_removed.connect(
    #lambda: print("CV/Audio B (two) jack removed"),
    # connect knob_x to set the x,z output socket ranges
    lambda: cv_magnitude_knob.value_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_knob.value_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_range),
    # and disconnect the socket from setting the x, z output ranges
    lambda: cv_magnitude_b_input_socket.value_changed.disconnect(lorenz_attractor_b_x_cv_output_socket.map_range),
    lambda: cv_magnitude_b_input_socket.value_changed.disconnect(lorenz_attractor_b_z_cv_output_socket.map_range),
    # update the range of the CV output to match the knob value
    lambda: lorenz_attractor_b_x_cv_output_socket.map_range(cv_magnitude_knob.ranged_variable),
    lambda: lorenz_attractor_b_z_cv_output_socket.map_range(cv_magnitude_knob.ranged_variable),
)

timestep_cv_input_socket = computer.cv_input_socket_one
timestep_cv_input_socket.set_voltage_range((-5, 5))

# if a jack is plugged into the timestep input socket,
timestep_cv_input_socket.jack_inserted.connect(
    #lambda: print("Timestep (one) jack inserted"),
    # disconnect the timestep knob from the timesteps of A and B
    lambda: timestep_knob.value_changed.disconnect(lorenz_attractor_a.timestep.map_value),
    lambda: timestep_knob.value_changed.disconnect(lorenz_attractor_b.timestep.map_value),
    # connect the timestep knob to the minimum value of the timesteps of A and B
    lambda: timestep_knob.value_changed.connect(lorenz_attractor_a.timestep.map_minimum_value),
    lambda: timestep_knob.value_changed.connect(lorenz_attractor_b.timestep.map_minimum_value),
    # TODO - force the timestep CV input socket to read any time that the main knob's value changes
    # connect the timestep input socket to the timesteps of A and B
    lambda: timestep_cv_input_socket.value_changed.connect(lorenz_attractor_a.timestep.map_value),
    lambda: timestep_cv_input_socket.value_changed.connect(lorenz_attractor_b.timestep.map_value),
    # update the timestep values to match the input (as it may be a static offset)
    lambda: lorenz_attractor_a.timestep.map_value(timestep_cv_input_socket.ranged_variable),
    lambda: lorenz_attractor_b.timestep.map_value(timestep_cv_input_socket.ranged_variable),
)

# if a jack is removed from the timestep input socket,
timestep_cv_input_socket.jack_removed.connect(
    #lambda: print("Timestep (CV one) jack removed"),
    # connect the timestep knob to the timesteps of A and B
    lambda: timestep_knob.value_changed.connect(lorenz_attractor_a.timestep.map_value),
    lambda: timestep_knob.value_changed.connect(lorenz_attractor_b.timestep.map_value),
    # disconnect the timestep knob from the min/max of the timesteps of A and B
    lambda: timestep_knob.value_changed.disconnect(lorenz_attractor_a.timestep.map_minimum_value),
    lambda: timestep_knob.value_changed.disconnect(lorenz_attractor_b.timestep.map_minimum_value),
    # and set the minimum timestep back to its default value - TODO
    #lambda: lorenz_attractor_a.timestep.minimum = 0.0001,
    #lambda: lorenz_attractor_b.timestep.minimum = 0.0001,
    # disconnect the timestep input socket from the timesteps of A and B
    lambda: timestep_cv_input_socket.value_changed.disconnect(lorenz_attractor_a.timestep.map_value),
    lambda: timestep_cv_input_socket.value_changed.disconnect(lorenz_attractor_b.timestep.map_value),
    # update the timestep to match the knob value
    lambda: lorenz_attractor_a.timestep.map_value(timestep_knob.ranged_variable),
    lambda: lorenz_attractor_b.timestep.map_value(timestep_knob.ranged_variable),
)

random_factor_cv_input_socket = computer.cv_input_socket_two
random_factor_cv_input_socket.set_voltage_range((0, 5))

random_factor_cv_input_socket.jack_inserted.connect(
    #lambda: print("Random factor (CV two) jack inserted"),
    # disconnect the random factor knob from the random factor of A and B
    lambda: random_factor_knob.value_changed.disconnect(lorenz_attractor_a.random_factor.map_value),
    lambda: random_factor_knob.value_changed.disconnect(lorenz_attractor_b.random_factor.map_value),
    # connect the random factor knob to the maximum value of the random factors of A and B
    lambda: random_factor_knob.value_changed.connect(lorenz_attractor_a.random_factor.map_maximum_value),
    lambda: random_factor_knob.value_changed.connect(lorenz_attractor_b.random_factor.map_maximum_value),
    # connect the random factor socket to the random factor of A and B
    lambda: random_factor_cv_input_socket.value_changed.connect(lorenz_attractor_a.random_factor.map_value),
    lambda: random_factor_cv_input_socket.value_changed.connect(lorenz_attractor_b.random_factor.map_value),
    # update the random factor to match the socket value
    lambda: lorenz_attractor_a.random_factor.map_value(random_factor_cv_input_socket.ranged_variable),
    lambda: lorenz_attractor_b.random_factor.map_value(random_factor_cv_input_socket.ranged_variable),
)

random_factor_cv_input_socket.jack_removed.connect(
    #lambda: print("Random factor (CV two) jack removed"),
    # connect the random factor knob to the random factor of A and B
    lambda: random_factor_knob.value_changed.connect(lorenz_attractor_a.random_factor.map_value),
    lambda: random_factor_knob.value_changed.connect(lorenz_attractor_b.random_factor.map_value),
    # disconnect the random factor knob from the maximum value of the random factors of A and B
    lambda: random_factor_knob.value_changed.disconnect(lorenz_attractor_a.random_factor.map_maximum_value),
    lambda: random_factor_knob.value_changed.disconnect(lorenz_attractor_b.random_factor.map_maximum_value),
    # disconnect the random factor socket from the random factor of A and B
    lambda: random_factor_cv_input_socket.value_changed.disconnect(lorenz_attractor_a.random_factor.map_value),
    lambda: random_factor_cv_input_socket.value_changed.disconnect(lorenz_attractor_b.random_factor.map_value),
    # update the random factor to match the knob value
    lambda: lorenz_attractor_a.random_factor.map_value(random_factor_knob.ranged_variable),
    lambda: lorenz_attractor_b.random_factor.map_value(random_factor_knob.ranged_variable),
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
        looper.toggle_looping
    )
)

# if switch Z is moved to the down position, set the initial coordinates of the looper
switch_z.switched_down.connect(looper.set_initial_coordinates)

# if a jack is inserted into the loop toggle pulse input,
loop_toggle_pulse_socket.jack_inserted.connect(
    # connect the socket to toggle looping if the switch is in the middle
    lambda: switch_z.is_middle and loop_toggle_pulse_socket.pulse_started.connect(
        looper.toggle_looping
    )
)

# if a jack is removed from the loop toggle pulse input,
loop_toggle_pulse_socket.jack_removed.connect(
    # disconnect the socket to prevent toggling of looping
    lambda: loop_toggle_pulse_socket.pulse_started.disconnect(
        looper.toggle_looping
    )
)

# if a jack is inserted into the set initial coordinates pulse input,
loop_set_initial_coordinates_socket.jack_inserted.connect(
    # connect the socket to set the initial coordinates
    lambda: loop_set_initial_coordinates_socket.pulse_started.connect(
        looper.set_initial_coordinates
    )
)

# if a jack is removed from the loop toggle pulse input,
loop_set_initial_coordinates_socket.jack_removed.connect(
    # disconnect the socket to prevent setting the initial coordinates
    lambda: loop_set_initial_coordinates_socket.pulse_started.disconnect(
        looper.set_initial_coordinates
    )
)

computer.update_input_sockets(fire_all_signals=True)
computer.led_matrix.turn_off()  # signal setup is complete

main_timer = TimerConnector(
    computer=computer,
    looper=looper,
    freq=MAXIMUM_TIMER_FREQUENCY_HZ,
)

counter = 0
while True:

    if counter == INPUT_SOCKET_UPDATE_STEPS:
        computer.update_input_sockets()
        counter = 0
    counter += 1

    computer.read_analog_inputs()
