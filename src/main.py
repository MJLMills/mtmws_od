import machine
import time
from lorenz_system import LorenzSystem
from looper import Looper
from irq_connectors import ToggleLooping, SetStartPoint
from timer_connectors import WriteOutput
from mapping import Mapping, MultiMapping
from ranged_value import RangedValue


led_pulse_output_length = 0.01

min_lorenz_system_scale_factor: float = 0.0
# min knob/CV settings will turn off CV.
max_lorenz_system_scale_factor: float = 1.0
# max knob/CV settings will set CV to max value.

timestep_min: float = 0.001
# min timestep has no mathematical limit, must be arbitrarily chosen.
timestep_max: float = 0.01
# max timestep is limited by the Euler method integration accuracy.
timestep_factor: float = 11 / 5
# sets how much slower B is than A
timestep_a =  RangedValue(max_value=timestep_max,
                          min_value=timestep_min)

timestep_b = RangedValue(max_value=timestep_max / timestep_factor,
                         min_value=timestep_min / timestep_factor)

divergence_min: float = 0.0
# min knob/CV settings will turn off sensitivity to initial conditions.
divergence_max: float = 1.0
# max divergence has no mathematical limit, must be arbitrarily chosen.
divergence = RangedValue(min_value=divergence_min,
                         max_value=divergence_max)

output_frequency_hz = 500
# write the outputs every 2000 microseconds

computer = Computer()
led_matrix = computer.led_matrix
led_matrix.turn_on()  # signal loading started to the user

lorenz_attractor_a = LorenzSystem(
    timestep=timestep_a,
    random_factor=divergence
)

lorenz_attractor_b = LorenzSystem(
    timestep=timestep_b,
    random_factor=divergence
)

looper = Looper(models=[lorenz_attractor_a,
                        lorenz_attractor_b])

led_id_pulse_output_a = 1
pulse_led_a = led_matrix.get_by_index(led_id_pulse_output_a)
led_id_pulse_output_b = 2
pulse_led_b = led_matrix.get_by_index(led_id_pulse_output_b)

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

# these mappings are from system variables to hardware outputs
lorenz_a_z_mapping = Mapping(source=lorenz_attractor_a.z,
                             output=cv_output_z_a)

lorenz_b_z_mapping = Mapping(source=lorenz_attractor_b.z,
                             output=cv_output_z_b)

lorenz_a_x_mapping = Mapping(source=lorenz_attractor_a.x,
                             output=cv_output_x_a)

lorenz_b_x_mapping = Mapping(source=lorenz_attractor_b.x,
                             output=cv_output_x_b)

write_output = WriteOutput(looper=looper,
                           lorenz_a_z_mapping=lorenz_a_z_mapping,
                           lorenz_b_z_mapping=lorenz_b_z_mapping,
                           lorenz_a_x_mapping=lorenz_a_x_mapping,
                           lorenz_b_x_mapping=lorenz_b_x_mapping,
                           pulse_output_a=pulse_output_a,
                           pulse_output_b=pulse_output_b,
                           pulse_led_a=pulse_led_a,
                           pulse_led_b=pulse_led_b,
                           freq=output_frequency_hz)

timestep_a_mapping = Mapping(
    source=timestep_knob,
    output=lorenz_attractor_a.timestep
)

timestep_b_mapping = Mapping(
    source=timestep_knob,
    output=lorenz_attractor_b.timestep
)

divergence_mapping_a = Mapping(
    source=divergence_knob,
    output=lorenz_attractor_a.random_factor
)

divergence_mapping_b = Mapping(
    source=divergence_knob,
    output=lorenz_attractor_b.random_factor
)

# the knob should set an overall magnitude from 0,1 (attenuating)
# at the output socket, you will only be able to write the output range ever, cannot go over max_value or under min_value
# one option is to limit the max output value based on the knob, then allow
# the CV to vary between the new min and max.
# this means knob -> (0.0, 1.0) -> mutiplication of x,z
# then CV -> (0.0, 1.0) multiplies the knob value
# this is a product then of the two scale factor inputs between 0 and 1

# where do these scale factors belong? if they both go into a single mapping
# as inputs, a mechanism is needed to combine them and map to the output range
scale_factor_knob_output = RangedValue(min_value=min_lorenz_system_scale_factor,
                                       max_value=max_lorenz_system_scale_factor)

scale_factor_cv_output = RangedValue(min_value=min_lorenz_system_scale_factor,
                                     max_value=max_lorenz_system_scale_factor)

scale_factor_mapping = MultiMapping(
    sources=[cv_magnitude_knob, cv_magnitude_cv_input],
    output=RangedValue(min_value=min_lorenz_system_scale_factor,
                       max_value=max_lorenz_system_scale_factor)
)

# pass these two mappings to another mapping that deals with them

time.sleep(0.5)  # make sure the loading indicator is visible
led_matrix.turn_off()  # signal loading complete to the user

# the code in the while loop runs whenever it can and will be paused when the
# timer and IRQ interrupts run.
while True:

    # do the analog reads (ints)
    computer.update_analog_inputs()

    # once analog reads are done, hardware -> software mappings can be updated too (floats)

    # there is a mapping from the scale-factor knob and CV to an output from (0, 1) which is
    # used to scale the Ax, Az, Bx, Bz values by a factor in the range [0, 1]
    # this is going to be another set of work - multiple sources combined to a single output
    scale_factor_mapping.update_latest_value()
    scale_factor_mapping.write()  # write it where?
    # multiply these, then map that input (0, 1) to the scale factor (who the hell owns the scale factor?)
    # feels like the scale factor is a property of the mapping from lorenz system to CV output sockets?

    # these two should not get out of sync, i.e. a timer between these steps is bad
    # perhaps these need to be on an interrupt
    timestep_a_mapping.update_latest_value(write_output=True)
    timestep_b_mapping.update_latest_value(write_output=True)

    divergence_mapping_a.update_latest_value(write_output=True)
    divergence_mapping_b.update_latest_value(write_output=True)

    if looper.needs_update:

        # this updates the raw coordinate values and crossing booleans
        looper.take_step()

        # this maps the raw coordinate values to their output ranges
        lorenz_a_z_mapping.update_latest_value()
        lorenz_b_z_mapping.update_latest_value()
        lorenz_a_x_mapping.update_latest_value()
        lorenz_b_x_mapping.update_latest_value()
