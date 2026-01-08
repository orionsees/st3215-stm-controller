/**
 * ST3215 Servo Terminal - Interactive command interface
 * Compile:   g++ -o servo_terminal servo_terminal.cpp -lpthread
 * Run:  sudo ./servo_terminal /dev/ttyACM0
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

const int NUM_SERVOS = 5;

struct ServoFeedback {
    int id = 0;
    int position = 0;
    int speed = 0;
    int load = 0;
    int temperature = 0;
    int voltage = 0;
    int current = 0;
    bool online = false;
};

int serial_fd = -1;
atomic<bool> running(true);
mutex feedback_mutex;
mutex print_mutex;
ServoFeedback servo_feedback[NUM_SERVOS];
bool show_debug = true;

void signalHandler(int) { running = false; }

int safeStoi(const string& str) {
    try { return str.empty() ? 0 : stoi(str); }
    catch (...) { return 0; }
}

vector<string> split(const string& str, char delim) {
    vector<string> tokens;
    stringstream ss(str);
    string token;
    while (getline(ss, token, delim)) tokens.push_back(token);
    return tokens;
}

int openSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;
    
    struct termios tty = {};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty. c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void sendCommand(const string& cmd) {
    string full = cmd + "\n";
    write(serial_fd, full.c_str(), full.length());
    tcdrain(serial_fd);
}

void parseFeedback(const string& line) {
    if (line.length() < 4 || line.substr(0, 3) != "$FB") return;
    auto tokens = split(line, ',');
    if (tokens.size() < 9) return;
    
    int id = safeStoi(tokens[1]);
    if (id < 1 || id > NUM_SERVOS) return;
    
    lock_guard<mutex> lock(feedback_mutex);
    int idx = id - 1;
    servo_feedback[idx]. id = id;
    servo_feedback[idx].position = safeStoi(tokens[2]);
    servo_feedback[idx].speed = safeStoi(tokens[3]);
    servo_feedback[idx]. load = safeStoi(tokens[4]);
    servo_feedback[idx].temperature = safeStoi(tokens[5]);
    servo_feedback[idx].voltage = safeStoi(tokens[6]);
    servo_feedback[idx].current = safeStoi(tokens[7]);
    servo_feedback[idx].online = true;
}

void receiverThread() {
    string line;
    char c;
    while (running) {
        if (read(serial_fd, &c, 1) > 0) {
            if (c == '\n') {
                if (! line.empty() && line[0] == '$') {
                    if (line.find("$FB") != string::npos) {
                        parseFeedback(line);
                    } else if (line. find("$DBG") != string::npos) {
                        if (show_debug) {
                            lock_guard<mutex> lock(print_mutex);
                            cout << "\r\033[K[STM32] " << line << endl << "> " << flush;
                        }
                    } else if (line.find("$ACK") != string::npos) {
                        lock_guard<mutex> lock(print_mutex);
                        cout << "\r\033[K[ACK] " << line << endl << "> " << flush;
                    } else if (line.find("$READY") != string::npos) {
                        lock_guard<mutex> lock(print_mutex);
                        cout << "\r\033[K[READY] STM32 Connected!" << endl << "> " << flush;
                    }
                }
                line.clear();
            } else if (c != '\r') {
                line += c;
            }
        }
    }
}

void printHelp() {
    cout << R"(
╔═══════════════════════════════════════════════════════════════════════╗
║                      ST3215 SERVO TERMINAL                            ║
╠═══════════════════════════════════════════════════════════════════════╣
║ POSITION CONTROL:                                                     ║
║   center                    - Move all servos to center (2048)        ║
║   pos <id> <pos> [spd] [acc]- Move one servo (spd=1000, acc=50)       ║
║   posall <pos> [spd] [acc]  - Move all servos to position             ║
║                                                                       ║
║ WHEEL MODE (Continuous Rotation):                                     ║
║   wheel <id>                - Set servo to wheel mode                 ║
║   servo <id>                - Set servo to position mode              ║
║   speed <id> <spd> [acc]    - Set wheel speed (-4095 to 4095)         ║
║   speedall <spd> [acc]      - Set all servos wheel speed              ║
║                                                                       ║
║ CONTROL:                                                               ║
║   stop                      - Emergency stop all motors               ║
║   torque <0/1>              - Disable/Enable torque                   ║
║                                                                       ║
║ INFO:                                                                 ║
║   status                    - Show servo positions                    ║
║   ping                      - Check connection                        ║
║   debug                     - Toggle debug messages                   ║
║   help                      - Show this help                          ║
║   quit                      - Exit                                    ║
╠═══════════════════════════════════════════════════════════════════════╣
║ EXAMPLES:                                                             ║
║   pos 1 2048                - Move servo 1 to center                  ║
║   pos 1 1000 500 100        - Move servo 1 to 1000 (spd=500, acc=100) ║
║   posall 2048 800           - Move all to 2048 at speed 800           ║
║   wheel 1                   - Set servo 1 to wheel mode               ║
║   speed 1 500               - Rotate forward at speed 500             ║
║   speed 1 -500              - Rotate backward at speed 500            ║
╚═══════════════════════════════════════════════════════════════════════╝
)" << endl;
}

void printStatus() {
    lock_guard<mutex> lock(feedback_mutex);
    
    cout << "\n+-----+---------+--------+--------+--------+--------+---------+" << endl;
    cout << "| ID  |   Pos   | Speed  |  Load  |  Temp  |  Volt  | Status  |" << endl;
    cout << "+-----+---------+--------+--------+--------+--------+---------+" << endl;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (servo_feedback[i].online) {
            printf("| %3d | %5d   | %5d  | %5d  |  %2d C  | %2d. %dV  |   OK    |\n",
                   servo_feedback[i].id,
                   servo_feedback[i]. position,
                   servo_feedback[i].speed,
                   servo_feedback[i].load,
                   servo_feedback[i].temperature,
                   servo_feedback[i].voltage / 10,
                   servo_feedback[i].voltage % 10);
        } else {
            printf("| %3d |   ---   |  ---   |  ---   |  ---   |  ---   | OFFLINE |\n", i + 1);
        }
    }
    cout << "+-----+---------+--------+--------+--------+--------+---------+\n" << endl;
}

void processCommand(const string& input) {
    if (input.empty()) return;
    
    vector<string> tokens = split(input, ' ');
    if (tokens.empty()) return;
    
    string cmd = tokens[0];
    for (char& c : cmd) if (c >= 'A' && c <= 'Z') c += 32;
    
    if (cmd == "help" || cmd == "? ") {
        printHelp();
    }
    else if (cmd == "quit" || cmd == "exit" || cmd == "q") {
        running = false;
    }
    else if (cmd == "debug") {
        show_debug = !show_debug;
        cout << "Debug messages: " << (show_debug ? "ON" : "OFF") << endl;
    }
    else if (cmd == "status" || cmd == "s") {
        printStatus();
    }
    else if (cmd == "ping") {
        cout << "Sending PING..." << endl;
        sendCommand("$CMD,PING");
    }
    else if (cmd == "center" || cmd == "c") {
        cout << "Moving all servos to center (2048)..." << endl;
        sendCommand("$CMD,CENTER");
    }
    else if (cmd == "stop") {
        cout << "EMERGENCY STOP!" << endl;
        sendCommand("$CMD,STOP");
    }
    else if (cmd == "torque") {
        if (tokens.size() >= 2) {
            string en = tokens[1];
            cout << "Setting torque: " << (en == "1" ? "ON" : "OFF") << endl;
            sendCommand("$CMD,TORQUE," + en);
        } else {
            cout << "Usage: torque <0/1>" << endl;
        }
    }
    // POS <id> <position> [speed] [acc]
    else if (cmd == "pos") {
        if (tokens.size() >= 3) {
            string id = tokens[1];
            string pos = tokens[2];
            string spd = tokens.size() >= 4 ? tokens[3] : "1000";
            string acc = tokens.size() >= 5 ? tokens[4] : "50";
            
            cout << "Moving servo " << id << " to " << pos 
                 << " (speed=" << spd << ", acc=" << acc << ")" << endl;
            sendCommand("$CMD,POS," + id + "," + pos + "," + spd + "," + acc);
        } else {
            cout << "Usage: pos <id> <position> [speed] [acc]" << endl;
        }
    }
    // POSALL <position> [speed] [acc]
    else if (cmd == "posall") {
        if (tokens.size() >= 2) {
            string pos = tokens[1];
            string spd = tokens.size() >= 3 ? tokens[2] : "1000";
            string acc = tokens. size() >= 4 ? tokens[3] : "50";
            
            cout << "Moving all servos to " << pos 
                 << " (speed=" << spd << ", acc=" << acc << ")" << endl;
            sendCommand("$CMD,POSALL," + pos + "," + spd + "," + acc);
        } else {
            cout << "Usage: posall <position> [speed] [acc]" << endl;
        }
    }
    // WHEEL <id>
    else if (cmd == "wheel") {
        if (tokens.size() >= 2) {
            cout << "Setting servo " << tokens[1] << " to WHEEL mode" << endl;
            sendCommand("$CMD,WHEEL," + tokens[1]);
        } else {
            cout << "Usage: wheel <id>" << endl;
        }
    }
    // SERVO <id>
    else if (cmd == "servo") {
        if (tokens.size() >= 2) {
            cout << "Setting servo " << tokens[1] << " to SERVO mode" << endl;
            sendCommand("$CMD,SERVO," + tokens[1]);
        } else {
            cout << "Usage: servo <id>" << endl;
        }
    }
    // SPEED <id> <speed> [acc]
    else if (cmd == "speed") {
        if (tokens.size() >= 3) {
            string id = tokens[1];
            string spd = tokens[2];
            string acc = tokens.size() >= 4 ? tokens[3] : "50";
            
            cout << "Setting servo " << id << " speed to " << spd 
                 << " (acc=" << acc << ")" << endl;
            sendCommand("$CMD,SPEED," + id + "," + spd + "," + acc);
        } else {
            cout << "Usage: speed <id> <speed> [acc]" << endl;
        }
    }
    // SPEEDALL <speed> [acc]
    else if (cmd == "speedall") {
        if (tokens.size() >= 2) {
            string spd = tokens[1];
            string acc = tokens.size() >= 3 ?  tokens[2] :  "50";
            
            cout << "Setting all servos speed to " << spd 
                 << " (acc=" << acc << ")" << endl;
            sendCommand("$CMD,SPEEDALL," + spd + "," + acc);
        } else {
            cout << "Usage:  speedall <speed> [acc]" << endl;
        }
    }
    // RAW command for debugging
    else if (cmd == "raw") {
        if (tokens.size() >= 2) {
            string raw = input.substr(4);
            cout << "Sending raw:  " << raw << endl;
            sendCommand(raw);
        } else {
            cout << "Usage: raw <command>" << endl;
        }
    }
    else {
        cout << "Unknown command: " << cmd << ". Type 'help' for commands." << endl;
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    const char* port = argc > 1 ?  argv[1] :  "/dev/ttyACM0";
    
    cout << "\n╔═══════════════════════════════════════════════════════════════════╗" << endl;
    cout << "║              ST3215 SERVO TERMINAL - Interactive Mode             ║" << endl;
    cout << "╚═══════════════════════════════════════════════════════════════════╝\n" << endl;
    cout << "Opening " << port << "..." << endl;
    
    serial_fd = openSerial(port);
    if (serial_fd < 0) {
        cerr << "Failed to open serial port!" << endl;
        return 1;
    }
    
    cout << "Connected!" << endl;
    cout << "Type 'help' for available commands.\n" << endl;
    
    thread receiver(receiverThread);
    this_thread::sleep_for(chrono:: seconds(2));
    
    string input;
    while (running) {
        {
            lock_guard<mutex> lock(print_mutex);
            cout << "> " << flush;
        }
        
        if (!getline(cin, input)) break;
        
        // Trim whitespace
        size_t start = input.find_first_not_of(" \t");
        size_t end = input.find_last_not_of(" \t");
        if (start != string:: npos) {
            input = input.substr(start, end - start + 1);
        } else {
            input.clear();
        }
        
        processCommand(input);
    }
    
    running = false;
    receiver.join();
    close(serial_fd);
    
    cout << "\nGoodbye!" << endl;
    return 0;
}