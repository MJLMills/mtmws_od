import machine
import time
from lorenz_system import LorenzSystem
from looper import Looper
from irq_connectors import ToggleLooping, SetStartPoint
from timer_connectors import WriteOutput
from mapping import Mapping

led_pulse_output_length = 0.01

timestep_factor: float = 11 / 5
# sets how much slower B is than A

min_lorenz_system_scale_factor: float = 0.0
# min knob/CV settings will turn off CV.
max_lorenz_system_scale_factor: float = 1.0
# max knob/CV settings will set CV to max value.

timestep_min: float = 0.001
# min timestep has no mathematical limit, must be arbitrarily chosen.
timestep_max: float = 0.01
# max timestep is limited by the Euler method integration accuracy.

divergence_min: float = 0.0
# min knob/CV settings will turn off sensitivity to initial conditions.
divergence_max: float = 1.0
# max divergence has no mathematical limit, must be arbitrarily chosen.

output_frequency_hz = 500
# write the outputs every 2000 microseconds

computer = Computer()
led_matrix = computer.led_matrix
led_matrix.turn_on()  # signal loading started to the user

lorenz_attractor_a = LorenzSystem()
lorenz_attractor_b = LorenzSystem()
looper = Looper(models=[lorenz_attractor_a,
                        lorenz_attractor_b])

led_id_pulse_output_a = 1
led_id_pulse_output_b = 2

# analog inputs
timestep_knob = computer.main_knob
cv_magnitude_knob = computer.knob_x
divergence_knob = computer.knob_y
loop_switch = computer.switch_z
timestep_cv_input = computer.cv_audio_input_socket_one
cv_magnitude_cv_input = computer.cv_audio_input_socket_two

# digital inputs
start_point_pulse_input = computer.pulses_input_socket_one
toggle_looping_pulse_input = computer.pulses_input_socket_two

# analog outputs
cv_output_z_a = computer.cv_audio_output_socket_one
cv_output_x_a = computer.cv_output_socket_one
cv_output_z_b = computer.cv_audio_output_socket_two
cv_output_x_b = computer.cv_output_socket_two

# digital outputs
pulse_output_a = computer.pulses_output_socket_one
pulse_output_b = computer.pulses_output_socket_two

# pulse input IRQs
set_start_point = SetStartPoint(
    pulse_input=start_point_pulse_input,
    looper=looper
)

toggle_looping = ToggleLooping(
    pulse_input=toggle_looping_pulse_input,
    looper=looper
)

# the integration step must only happen once before each trigger of the output timer

write_output = WriteOutput(lorenz_system_a=lorenz_attractor_a,
                           lorenz_system_b=lorenz_attractor_b,
                           cv_output_x_a=cv_output_x_a,
                           cv_output_z_a=cv_output_z_a,
                           cv_output_x_b=cv_output_x_b,
                           cv_output_z_b=cv_output_z_b,  # TODO - add pulse outputs
                           freq=output_frequency_hz)

scale_factor_knob_mapping = Mapping(
    input_=cv_magnitude_knob,
    output_min=min_lorenz_system_scale_factor,
    output_max=max_lorenz_system_scale_factor
)

scale_factor_cv_mapping = Mapping(
    input_=cv_magnitude_cv_input,
    output_min=min_lorenz_system_scale_factor,
    output_max=max_lorenz_system_scale_factor
)

timestep_a_mapping = Mapping(
    input_=timestep_knob,
    output_min=timestep_min,
    output_max=timestep_max
)

timestep_b_mapping = Mapping(
    input_=timestep_knob,
    output_min=timestep_min / timestep_factor,
    output_max=timestep_max / timestep_factor
)

divergence_mapping = Mapping(
    input_=divergence_knob,
    output_min=divergence_min,
    output_max=divergence_max
)

time.sleep(0.5)  # make sure the loading indicator is visible
led_matrix.turn_off()  # signal loading complete to the user

# the code in the while loop runs whenever it can and will be paused when the
# timer and IRQ interrupts run.
while True:

    computer.update_analog_inputs()
    # once inputs are updated, mappings can be updated too (floats)
    # TODO - make this another function call so it can be deferred later or not

    # once mappings are updated, their outputs can be too
    lorenz_attractor_a.scale_factor = scale_factor_knob_mapping.current_value + scale_factor_cv_mapping
    lorenz_attractor_b.scale_factor = scale_factor_knob_mapping.current_value + scale_factor_cv_mapping
    lorenz_attractor_a.timestep = timestep_a_mapping.current_value
    lorenz_attractor_b.timestep = timestep_b_mapping.current_value
    lorenz_attractor_a.random_factor = divergence_mapping.current_value
    lorenz_attractor_b.random_factor = divergence_mapping.current_value

    # there is a mapping from the Lorenz system outputs to the CV and CV/Audio output sockets
    if looper.needs_update:
        lorenz_attractor_a.take_step()
        lorenz_attractor_b.take_step()
