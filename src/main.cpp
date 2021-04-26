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

    void safe_off() {
        pwm.pulsewidth(0);
    }

 };

/** Simple class for simple surface "Submarine" with 2 motor **/
class SUB {
private:
    ESC& esc_left;
    ESC& esc_right;
    int yaw_effort{0};
    int thrust_effort{500};

public:
    SUB(ESC& left, ESC& right) : esc_left{left}, esc_right{right} {
        /** Some ESCs requires a initial "neutral" throttle signal to initialize **/
    }

    void init() {
        esc_left.init();
        esc_right.init();
    }

    void set_thrust_effort(int thrust) {
        if (thrust > 1000) {
            thrust_effort = 1000;
        } else if (thrust < 0) {
            thrust_effort = 0;
        } else {
            thrust_effort = thrust;
        }
    }

    void set_yaw_effort(int yaw) {
        if (yaw > 1000) {
            yaw_effort = 1000;
        } else if (yaw < -1000) {
            yaw_effort = -1000;
        } else {
            yaw_effort = yaw;
        }
    }

    int get_thrust_effort() const {
        return thrust_effort;
    }

    int get_yaw_effort() const {
        return yaw_effort;
    }

    void update() {
        esc_left.setThrottle(thrust_effort - yaw_effort);
        esc_right.setThrottle(thrust_effort + yaw_effort);
        esc_left.update();
        esc_right.update();
    }

    void safe_off() {
        esc_left.safe_off();
        esc_right.safe_off();
    }

};


class Link {
private:
    uint64_t heartbeat{0};
    char buffer_in[256];
    char buffer_out[256];
    StaticJsonDocument<JSON_OBJECT_SIZE(10)> in;        // max 10 key-value pairs
    StaticJsonDocument<JSON_OBJECT_SIZE(10)> out;

public:
    JsonObject packet_in = in.to<JsonObject>();
    JsonObject packet_out = out.to<JsonObject>();

    Link() {}
    
    void send_packet(JsonObject& packet, const char* target="") {
        packet["sender"] = "nucleo";
        packet["target"] = target;
        serializeJson(packet, buffer_out, sizeof(buffer_out));
        printf("%s\n", buffer_out);
    }

    void send_ack() {
        out.clear();
        packet_out["ack"] = true;
        send_packet(packet_out);
    }

    void send_log(const char* level, const char* msg, const char* echo="", const char* target="") {
        out.clear();
        packet_out["level"] = level;
        packet_out["log"] = msg;
        packet_out["echo"] = echo;
        send_packet(packet_out);
    }

    int get_packet() {
        if (serial.readable()) {
                fgets(buffer_in, sizeof(buffer_in), stdin);
                DeserializationError err = deserializeJson(in, buffer_in, sizeof(buffer_in));
                if (err) {
                    send_log("warning", "Msg recv: corrupt or unknown format", buffer_in);
                } else if (strcmp(packet_in["target"], "nucleo")) {
                    send_log("error", "Msg recv: wrong [target]");
                } else {
                    return 0;
                }
        }
        return -1;
    }
};


PwmOut pwm[2] = {{PC_6}, {PC_8}};
ESC esc[2] = {ESC(pwm[0]), ESC(pwm[1])};
SUB sub(esc[0], esc[1]);
Link link;

int watchdog = 0;
bool arm = false;

void wd_reset() {
    watchdog = 10; // 10 cycles, about 1 secs
}

void handshake(Link& link) {
    while (true) {
        int err = link.get_packet();
        if (!err) {
            arm = link.packet_in["arm"] | false;    // default to false if not exist
            if (arm) {
                sub.init();
                for (int i = 0; i < 5; i++) {       // keep link activity to stabilize serial link, while waiting for esc to be ready
                    link.get_packet();
                    link.send_ack();
                    wait_us(1*sec);
                }
                wd_reset();
                link.send_log("info", "Ready to Rumble");
            }
        }

        link.send_log("info", "Waiting for arm message");
        wait_us(1*sec);
    }
}

int main() {
    while(1) {
        //Handshaking at initialization
        if (!arm) {
            handshake(link);    
        }

        // Control loop
        int err = link.get_packet();
        if (err) {
            // one period with wrong message
            watchdog--;

        } else {
            int thrust = link.packet_in["thrust"] | 500;   //default to 500 if wrong value type
            int yaw = link.packet_in["yaw"] | 0;         //default to 0 if key does not exist or wrong value type
            sub.set_thrust_effort(thrust);
            sub.set_yaw_effort(yaw);
            sub.update();
            wd_reset();
            link.send_ack();
        }

        if (watchdog < 0) {        
            sub.safe_off();
            arm = false;    
            link.send_log("error", "Watchdog: communication interrupted. Safe off triggered");
        }

        if (link.packet_in["disarm"] == true) {                
            sub.safe_off();
            arm = false;
            link.send_log("warn", "Safe off command received");
        }

        wait_us(100*msec); // 10Hz
    }

}