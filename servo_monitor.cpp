/**
 * ST3215 Servo Monitor - Real-time feedback display
 * Compile:   g++ -o servo_monitor servo_monitor.cpp -lpthread
 * Run:  sudo ./servo_monitor /dev/ttyACM0
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
    int moving = 0;
    bool online = false;
};

int serial_fd = -1;
atomic<bool> running(true);
mutex feedback_mutex;
ServoFeedback servo_feedback[NUM_SERVOS];

void signalHandler(int) { running = false; }

int safeStoi(const string& str) {
    try { return str.empty() ? 0 : stoi(str); } 
    catch (... ) { return 0; }
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
    tty. c_lflag = 0;
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
    servo_feedback[idx].moving = safeStoi(tokens[8]);
    servo_feedback[idx]. online = true;
}

void receiverThread() {
    string line;
    char c;
    while (running) {
        if (read(serial_fd, &c, 1) > 0) {
            if (c == '\n') {
                if (! line.empty() && line[0] == '$') {
                    if (line.find("$FB") != string::npos) parseFeedback(line);
                    else if (line.find("$DBG") != string::npos) cout << "[STM32] " << line << endl;
                    else if (line.find("$ACK") != string::npos) cout << "[ACK] " << line << endl;
                }
                line.clear();
            } else if (c != '\r') {
                line += c;
            }
        }
    }
}

void displayFeedback() {
    lock_guard<mutex> lock(feedback_mutex);
    cout << "\033[2J\033[H";  // Clear screen
    
    cout << "╔═══════════════════════════════════════════════════════════════╗" << endl;
    cout << "║          ST3215 SERVO MONITOR - Real-time Feedback            ║" << endl;
    cout << "╠═════╦═════════╦════════╦════════╦════════╦════════╦═══════════╣" << endl;
    cout << "║ ID  ║   Pos   ║ Speed  ║  Load  ║  Temp  ║  Volt  ║  Status   ║" << endl;
    cout << "╠═════╬═════════╬════════╬════════╬════════╬════════╬═══════════╣" << endl;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (servo_feedback[i]. online) {
            printf("║ %3d ║ %5d   ║ %5d  ║ %5d  ║  %2d C  ║ %2d. %dV  ║    OK     ║\n",
                   servo_feedback[i].id,
                   servo_feedback[i].position,
                   servo_feedback[i]. speed,
                   servo_feedback[i].load,
                   servo_feedback[i].temperature,
                   servo_feedback[i].voltage / 10,
                   servo_feedback[i].voltage % 10);
        } else {
            printf("║ %3d ║   ---   ║  ---   ║  ---   ║  ---   ║  ---   ║  OFFLINE  ║\n", i + 1);
        }
    }
    cout << "╚═════╩═════════╩════════╩════════╩════════╩════════╩═══════════╝" << endl;
    cout << "\nPress Ctrl+C to exit" << endl;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    const char* port = argc > 1 ?  argv[1] :  "/dev/ttyACM0";
    
    cout << "Opening " << port << "..." << endl;
    serial_fd = openSerial(port);
    if (serial_fd < 0) { cerr << "Failed!" << endl; return 1; }
    
    thread receiver(receiverThread);
    this_thread::sleep_for(chrono:: seconds(2));
    
    while (running) {
        displayFeedback();
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    
    running = false;
    receiver.join();
    close(serial_fd);
    return 0;
}