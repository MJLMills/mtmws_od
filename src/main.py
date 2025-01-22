MINIMUM_TIMESTEP: float = 0.0001
"""min timestep has no mathematical limit, must be arbitrarily chosen."""

MAXIMUM_TIMESTEP: float = 0.01
"""max timestep is limited by the Euler method integration accuracy."""

TIMESTEP_FACTOR: float = 5/11
"""sets how much slower B is than A."""

DIVERGENCE_MIN: float = 0.0
"""min knob/CV settings will turn off sensitivity to initial conditions."""

DIVERGENCE_MAX: float = 5.0
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
lorenz_attractor_a.crossed_zero.connect(crossed_zero_pulse_output_socket_a.pulse)
lorenz_attractor_a.x_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_and_write_value)
lorenz_attractor_a.z_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_and_write_value)

main_knob.value_changed.connect(lorenz_attractor_a.timestep.map_value)
computer.knob_y.value_changed.connect(lorenz_attractor_a.random_factor.map_value)

# lorenz B

crossed_zero_led_b = computer.led_matrix.get_by_index(2)
crossed_zero_pulse_output_socket_b = computer.pulses_output_socket_two

lorenz_attractor_b_x_cv_output_socket = computer.cv_output_socket_two
lorenz_attractor_b_z_cv_output_socket = computer.cv_audio_output_socket_two

lorenz_attractor_b.crossed_zero.connect(crossed_zero_led_b.pulse)
lorenz_attractor_b.crossed_zero.connect(crossed_zero_pulse_output_socket_b.pulse)
lorenz_attractor_b.x_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_and_write_value)
lorenz_attractor_b.z_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_and_write_value)

main_knob.value_changed.connect(lorenz_attractor_b.timestep.map_value)
computer.knob_y.value_changed.connect(lorenz_attractor_b.random_factor.map_value)

# hardware
# Knob X acts as a VCA on all four output CVs

knob_x.value_changed.connect(lorenz_attractor_a_x_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_a_z_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_b_x_cv_output_socket.map_range)
knob_x.value_changed.connect(lorenz_attractor_b_z_cv_output_socket.map_range)

# looper

looper_status_led = computer.led_matrix.get_by_index(3)
set_start_led = computer.led_matrix.get_by_index(4)
switch_z = computer.switch_z
loop_toggle_pulse_socket = computer.pulses_input_socket_one
loop_set_initial_coordinates_socket = computer.pulses_input_socket_two

if switch_z.is_middle():
    loop_toggle_pulse_socket.pulse_started.connect(looper.toggle_looping)
elif switch_z.is_up():
    looper.start_looping()

loop_set_initial_coordinates_socket.pulse_started.connect(looper.set_initial_coordinates)

switch_z.switched_up.connect(looper.start_looping)
switch_z.switched_up.connect(lambda: loop_toggle_pulse_socket.pulse_started.disconnect(looper.toggle_looping))

switch_z.switched_up_to_middle.connect(looper.stop_looping)
switch_z.switched_up_to_middle.connect(lambda: loop_toggle_pulse_socket.pulse_started.connect(looper.toggle_looping))

switch_z.switched_down.connect(looper.set_initial_coordinates)

looper.looping_started.connect(looper_status_led.turn_on)
looper.looping_stopped.connect(looper_status_led.turn_off)
looper.initial_coordinates_set.connect(set_start_led.pulse)


write_output = TimerConnector(
    looper=looper,
    freq=225,  # 225
)

while True:
    computer.read_analog_inputs()