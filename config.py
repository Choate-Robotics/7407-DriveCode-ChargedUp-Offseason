from robotpy_toolkit_7407.motors.rev_motors import SparkMax, SparkMaxConfig

left_intake_motor_id = 15  # change this to the left intake motor id
right_intake_motor_id = 16  # change this to the right intake motor id
default_intake_speed = 0.5  # change this to the default intake speed
wrist_motor_id = 17  # change this to the wrist motor id
INTAKE_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=1)
WRIST_CONFIG = SparkMaxConfig(k_P=1, k_I=1, k_D=1, k_F=1)
disable_wrist_rotation = False  # change this to disable wrist rotation