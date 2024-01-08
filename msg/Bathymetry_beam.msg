uint32 sample_number        # Sample number 
float32 angle               # Angle [rad] 
uint16 upper_gate           # Sample number for adaptive gate 
uint16 lower_gate           
float32 intensity           # Intensity for beam
uint16 flags                # Info flags for beam
uint8 quality_flag          # Quality flags for beam
uint8 quality_val           # Quality value for beam 

float32 range # Computed in ROS as sample_number*snd_velocity/(2*sample_rate)