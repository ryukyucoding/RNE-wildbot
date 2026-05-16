#define FB_MOTOR 20010
#define FB_INFO	 20011

// {"T":10000,"id":1}
// wheel_stop(id)
#define CMD_WHEEL_STOP	10000

// {"T":10010,"id":1,"cmd":50,"act":3}
// wheel_ctrl(id, cmd, act)
#define CMD_WHEEL_CTRL 10010

// 1: current loop
// 2: speed loop
// 3: position loop
// {"T":10012,"id":1,"mode":2}
// wheel_change_mode(id, mode)
#define CMD_WHEEL_CHANGE_MODE	10012


// get other info
// {"T":10032,"id":1}
// wheel_get_info(id)
#define CMD_WHEEL_INFO	10032

// {"T":11001,"time":2000}
// {"T":11001,"time":-1}
// set_heartbeat_time(time_ms)
#define CMD_HEARTBEAT_TIME	11001



// === === === esp32 settings. === === ===

// reboot device.
// {"T":600}
#define CMD_REBOOT 600

