#include <mbed.h>
#include <ArduinoJson.h>

BufferedSerial serial(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd)     // necessary for printf, stdin, debug to work
{
    return &serial;
}

const int sec = 1000000; // usec
const int msec = 1000;   // usec

/** Simple class for ESC **/
class ESC {    
private:  
    PwmOut& pwm;
    int period;
    int duty;
    const int min_duty{1000};
    const int max_duty{2000};
    
public:
    ESC(PwmOut& _pwm, int period_ms=20) : pwm{_pwm}, period{period_ms} {
        pwm.period_ms(20);
    }

    void init() {
        setThrottle(500);
        update();
    }

    /** throttle input: 0 (max reverse), 1000 (max forward), 500 (stop) **/
    void setThrottle(int t) {
        if (t > 1000) {
            t = 1000;
        } else if (t < 0) {
            t = 0;
        }
        duty = 1000 + t;
    }
    
    int getThrottle() const {
        return duty - 1000;
    }
    
    void update(){
        pwm.pulsewidth_us(duty);
    }

    void safeOff() {
        pwm.pulsewidth(0);
    }

 };

/** Simple class for simple surface "Submarine" with 2 motor **/
class SUB {
private:
    ESC& esc_left;
    ESC& esc_right;
    bool _arm{false};
    int yaw_effort{0};
    int pitch_effort{500};

public:
    SUB(ESC& left, ESC& right) : esc_left{left}, esc_right{right} {
        /** Some ESCs requires a initial "neutral" throttle signal to initialize **/
    }

    void setPitchEffort(int pitch) {
        if (pitch > 1000) {
            pitch_effort = 1000;
        } else if (pitch < -1000) {
            pitch_effort = -1000;
        } else {
            pitch_effort = pitch;
        }
    }

    void setYawEffort(int yaw) {
        if (yaw > 1000) {
            yaw_effort = 1000;
        } else if (yaw < -1000) {
            yaw_effort = -1000;
        } else {
            yaw_effort = yaw;
        }
    }

    int getPitchEffort() const {
        return pitch_effort;
    }

    int getYawEffort() const {
        return yaw_effort;
    }

    void update() {
        esc_left.setThrottle((pitch_effort+1000)/2 - yaw_effort);
        esc_right.setThrottle((pitch_effort+1000)/2 + yaw_effort);
        esc_left.update();
        esc_right.update();
    }

    void arm() {
        esc_left.init();
        esc_right.init();
        _arm = true;
    }

    void disarm() {
        esc_left.safeOff();
        esc_right.safeOff();
        _arm = false;
    }

    bool isArmed() {
        return _arm;
    }

};

/** Simple class for JSON communication over serial link */
class Link {
private:
    uint64_t heartbeat{0};
    char buffer_in[256];
    char buffer_out[256];
    StaticJsonDocument<JSON_OBJECT_SIZE(10)> in;        // max 10 key-value pairs
    StaticJsonDocument<JSON_OBJECT_SIZE(10)> out;

public:

    JsonObject packet_in = in.to<JsonObject>();
   
    void sendPacket(JsonObject& packet, const char* target="") {
        packet["sender"] = "nucleo";
        packet["target"] = target;
        serializeJson(packet, buffer_out, sizeof(buffer_out));
        printf("%s\n", buffer_out);
    }

    void sendAck() {
        JsonObject packet_out = out.to<JsonObject>();
        packet_out.clear();
        packet_out["type"] = "ack";
        sendPacket(packet_out);
    }

    void sendLog(const char* level, const char* msg, const char* echo="", const char* target="") {
        JsonObject packet_out = out.to<JsonObject>();
        packet_out["type"] = "log";
        packet_out["level"] = level;
        packet_out["log"] = msg;
        packet_out["echo"] = echo;
        sendPacket(packet_out);
    }

    // TODO: fgets blocks if there is no serial data to read. Need to put it on a thread. Otherwise the control loop can hang.
    int getPacket(const char* type) {
        if (serial.readable()) {
                fgets(buffer_in, sizeof(buffer_in), stdin);
                DeserializationError err = deserializeJson(in, buffer_in, sizeof(buffer_in));
                if (err) {
                    sendLog("debug", "Msg recv: corrupt or unknown format", buffer_in);

                // TODO: these strcmp conditions are quite wasteful, consider using ID instead of strings
                } else if (!packet_in.containsKey("target") || !packet_in["target"].is<const char*>() || strcmp(packet_in["target"], "nucleo")) {
                    sendLog("debug", "Msg recv: wrong [target]");

                } else if (!packet_in.containsKey("type") || !packet_in["type"].is<const char*>() || strcmp(packet_in["type"], type)) {
                    sendLog("debug", "Packet recv: unexpected packet type");

                } else {
                    return 0;
                }
        }
        return -1;
    }
};

/** Global variables */
PwmOut pwm[2] = {{PC_6}, {PC_8}};
ESC esc[2] = {ESC(pwm[0]), ESC(pwm[1])};
SUB sub(esc[0], esc[1]);
Link link;
int watchdog = 0;

/** Helper functions */
void wd_reset() {
    watchdog = 10; // 10 cycles, about 1 secs
}


/** infinite loop until armed **/
void handshake(Link& link) {
    while (true) {
        serial.sync();   // flush buffer from stale data
        int err = link.getPacket("handshake");
        if (!err) {
            if (link.packet_in["arm"] | false) {    // default to false if not exist
                sub.arm();
                for (int i = 0; i < 50; i++) {       // keep link activity to stabilize serial link, while waiting for esc to be ready
                    link.getPacket("handshake");
                    wait_us(100*msec);
                }
                wd_reset();
                link.sendAck();
                link.sendLog("info", "Armed");
                break;
            }
        }

        link.sendLog("info", "Waiting for arm message");
        wait_us(1*sec);
    }
}

int main() {
    while(1) {
        //Handshaking at initialization
        if (!sub.isArmed()) {
            handshake(link);
        }

        // Control loop
        int err = link.getPacket("cmd");
        if (err) {
            // one period with wrong message
            watchdog--;

        } else {
            int pitch = link.packet_in["pitch"] | 500;   //default to 500 if wrong value type
            int yaw = link.packet_in["yaw"] | 0;         //default to 0 if key does not exist or wrong value type
            sub.setPitchEffort(pitch);
            sub.setYawEffort(yaw);
            sub.update();
            wd_reset();
            link.sendAck();
        }

        if (watchdog < 0) {   
            sub.disarm();    
            link.sendLog("info", "Watchdog: communication interrupted. Disarmed");
        }

        if (link.packet_in["disarm"] | false) {
            sub.disarm();    
            link.sendLog("warning", "Disarmed");
        }

        wait_us(100*msec); // 10Hz
    }

}