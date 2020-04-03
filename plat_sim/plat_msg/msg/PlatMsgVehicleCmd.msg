# Data CMD
float64 vel_cmd                 # VELocity
float64 swa_cmd                 # Steering Wheel Angle
float64 acc_cmd                 # ACCeleration
float64 sar_cmd                 # Steering wheel Angular Rate

# Config CMD
uint8 cfg_type_cmd

# Enable CMD
bool enable_cmd

# Clear CMD
bool clear_cmd

# CMD CFG Types
uint8 CMD_NONE=0                #0000
uint8 CMD_VEL=1                 #0001
uint8 CMD_SWA=2                 #0010
uint8 CMD_VEL_SWA=3             #0011
uint8 CMD_ACC=4                 #0100
uint8 CMD_VEL_ACC=5             #0101
uint8 CMD_ACC_SWA=6             #0110
uint8 CMD_VEL_SWA_ACC=7         #0111
uint8 CMD_SAR=8                 #1000
uint8 CMD_VEL_SAR=9             #1001
uint8 CMD_SWA_SAR=10            #1010
uint8 CMD_VEL_SWA_SAR=11        #1011
uint8 CMD_ACC_SAR=12            #1100
uint8 CMD_VEL_ACC_SAR=13        #1101
uint8 CMD_SWA_ACC_SAR=14        #1110
uint8 CMD_VEL_ACC_SWA_SAR=15    #1111

# Maximum Values
float64 VEL_MAX=1.0
float64 VEL_MIN=0.0
float64 SWA_MAX=1.0
float64 SWA_MIN=-1.0
float64 ACC_MAX=1.0
float64 ACC_MIN=-1.0
float64 SAR_MAX=1.0
float64 SAR_MIN=-1.0