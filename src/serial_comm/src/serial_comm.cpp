#include "serial_comm.hpp"

double mapValues(double data, double in_min, double in_max, double out_min, double out_max){   
  return ((((data - in_min)*(out_max - out_min))/(in_max - in_min)) + out_min);
}

SerialComm::SerialComm(): Node("joy_node"){
    initParam();
    initPort();
    msg.axes.resize(2);
    msg.buttons.resize(2);
    memset(data.buffer, 0, sizeof(data.buffer));
    joy_pub = this->create_publisher<JoyMsg>("joy", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(),
                                    std::bind(&SerialComm::dataRead, this));
}

SerialComm::~SerialComm(){
    close(device.port);
}

bool SerialComm::initPort(){
    device.port = open(device.file_name.c_str(), O_RDWR);
    std::cout << device.file_name.c_str() << std::endl;
    if(device.port < 0){
        std::cout << "Error " << errno << " from open : " << strerror(errno) << std::endl;
        return false;
    }
    if(!configure()){
        std::cout << "Error configure..." << std::endl;
        return false;
    }
    usleep(1000000);
    return true;
}

bool SerialComm::configure(){
    struct termios tty;
    if(tcgetattr(device.port, &tty) != 0){
        std::cout << "Error " << errno << " from tcgetattr : " << strerror(errno) << std::endl;
        close(device.port);
        return false;
    }
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;  
    tty.c_lflag &= ~ECHOE; 
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG; 

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;  
    tty.c_cc[VMIN] = 1;

    cfsetspeed(&tty, BAUDRATE);
    tcflush(device.port, TCIFLUSH);
    if (tcsetattr(device.port, TCSANOW, &tty) != 0) {
        std::cout << "Error " << errno << " from tcsetattr : " << strerror(errno) << std::endl;
        close(device.port);
        return false;
    }

    return true;
}

void SerialComm::dataRead(){
    while (read(device.port, &data.curr_byte, 1) > 0){
        if(data.state == 0){
            if(data.curr_byte == HEADER_ && data.prev_byte == FOOTHER_){
                data.buffer[data.state++] = data.curr_byte;
            }else{
                data.state = 0;
            }
        }else if(data.state < HEADER_LEN + PAYLOAD_LEN){
            data.buffer[data.state++] = data.curr_byte;
        }else if(data.state < HEADER_LEN + PAYLOAD_LEN + FOOTHER_LEN){
            data.buffer[data.state++] = data.curr_byte;
            data.state = 0;
            data.prev_byte = data.curr_byte;
            if(data.curr_byte == FOOTHER_){
                msg.axes[1]    = mapValues(static_cast<float>(data.buffer[JOY_X]), 0, 200, -1, 1);
                msg.axes[0]    = mapValues(static_cast<float>(data.buffer[JOY_Y]), 0, 200, -1, 1);
                msg.buttons[1] = static_cast<int>(data.buffer[OBS_FLAG]);
                msg.buttons[0] = static_cast<int>(data.buffer[ARM_FLAG]);
                joy_pub->publish(msg);
            }else{
                data.state = 0;
            }
        }else{
            data.state = 0;
        }
        data.prev_byte = data.curr_byte;  
    }
}

void SerialComm::initParam(){
    this->declare_parameter<std::string>("file_name", "/dev/ttyUSB0");
    device.file_name = this->get_parameter("file_name").as_string();
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialComm>());
    rclcpp::shutdown();
    return 0;
}