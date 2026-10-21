TeachPoint[] points
bool loop_active
int32 cycle
string current_point        # point being moved to, empty when idle
bool auto_mode              # TCP-sensor auto-run armed (runs a pass per part seen at home)
int32 pending_parts         # unused with the TCP sensor (no queue); kept for compatibility
int32 auto_input_bit        # which input the auto-run watches (default IN1, TCP sensor)
