#Header header 
float32[3] angle # degrees
float32[3] vel # deg/s
float32[3] current # amps
#time armReadTimestamp 
time startSyncTime 
uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
uint8 bufferHealth
