# Yellow area in Norbit Technical Document
uint32 preamble
uint32 type                 # Set to 1 for bathymetric data
uint32 size

# Green area
float32 snd_velocity
float32 sample_rate 
uint32 num_beams
uint32 ping_number          # Sequence number of ping 
float64 time                # Ping timestamp, unix time at tx
float64 time_net            # Timestamp Unix time as fract (send on network time)
float32 ping_rate           # Hz
uint16  bathy_type          # Bathy data type 
uint8   beam_dist_mode      # 1:512EA 2:256EA, equiangular
uint8   sonar_mode          # 0:FLS 1:Bathy 2:Bathy amp 3:TBD 
                            # 4:bf pass- through 5:Bathy edge enhance 
                            # 6: Super res bathy
float32 tx_angle            # Tx elevation steering in radians
float32 gain                # Intensity value gain
float32 tx_freq            
float32 tx_bw
float32 tx_len
float32 tx_voltage          # Peak voltage, NaN on non-STX sonars
float32 swath_dir           # Center pointing direction of swath [rad]
float32 swath_open          # Opening angle of swath [rad]
float32 gate_tilt           # Gate tilt [rad], for depth gates

# Blue area
norbit_wbms_driver/Bathymetry_beam[] beams