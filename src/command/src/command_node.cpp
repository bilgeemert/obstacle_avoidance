#include "command_node.hpp"

Command::Command(): Node("command_node"){
    initParam();
    controlSelection();
    joy_data.axes.resize(2);
    memset(data.buffer, 0, sizeof(data.buffer));
}

void Command::keyboardControl(){
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    elapsed_time = current_time - last_msg_timestamp_;

     if (elapsed_time.count() > 100000000) {
        command_pub->publish(joy_data);
    } 
}


void Command::controlSelection(){
    if(control_unit == "joy"){
        joy_sub = this->create_subscription<joyMsg>("joy", 10,
                        std::bind(&Command::joyCallback, this, std::placeholders::_1));

    } else if(control_unit == "keyboard"){ 
        keyboard_sub = this->create_subscription<int32Msg>("/keypress", 10,
                std::bind(&Command::keyboardCallback, this, std::placeholders::_1));
        keyboard_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                        std::bind(&Command::keyboardControl, this)); 

    } else if(control_unit == "esp8266"){
        if(initPort()){
            timer_ = this->create_wall_timer(std::chrono::milliseconds(),
                                        std::bind(&Command::dataRead, this));                          
        }
    } else{
        std::cout << "control off" << std::endl;
    }
    command_pub = this->create_publisher<joyMsg>("command_data", 10);
}

Command::~Command(){
    close(device.port);
}

bool Command::initPort(){
    device.port = open(device.file_name.c_str(), O_RDWR);
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

bool Command::configure(){ // Serial
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

void Command::dataRead(){ // esp8266 serial
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
                joy_data.axes[0] = map(static_cast<float>(data.buffer[JOY_Y]), 0, 200, -1, 1);
                joy_data.axes[1] = map(static_cast<float>(data.buffer[JOY_X]), 0, 200, 1, -1);
                command_pub->publish(joy_data);
            }else{
                data.state = 0;
            }
        }else{
            data.state = 0;
        }
        data.prev_byte = data.curr_byte;  
    }
}

void Command::initParam(){
    this->declare_parameter<std::string>("file_name", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("control_unit", "keyboard");
    device.file_name = this->get_parameter("file_name").as_string();
    control_unit = this->get_parameter("control_unit").as_string();
}

void Command::keyboardCallback(const int32Msg msg){
    bool is_ready = true;
    joy_data.axes[0] = 0.0;
    joy_data.axes[1] = 0.0;

    switch(msg.data){
      case KEYCODE_W:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        joy_data.axes[0] = 1.0;
        break;
      case KEYCODE_A:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        joy_data.axes[1] = 0.5;
        break;
      case KEYCODE_S:
        RCLCPP_DEBUG(this->get_logger(), "STOP");
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        joy_data.axes[1] = -0.5;
        break;
      case KEYCODE_X:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        joy_data.axes[0] = -1.0;
        break;
      default:
        RCLCPP_DEBUG(this->get_logger(), "None: %x", msg.data);
        is_ready = false;
        break;
    }

    if(is_ready){
        last_msg_timestamp_ = std::chrono::steady_clock::now();
        command_pub->publish(joy_data);
    }
}

void Command::joyCallback(const joyMsg msg){
    joy_data.axes[0] = msg.axes[1]; 
    joy_data.axes[1] = msg.axes[0];
    command_pub->publish(joy_data);
}

double map(double data, double in_min, double in_max, double out_min, double out_max){   
  return ((((data - in_min)*(out_max - out_min))/(in_max - in_min)) + out_min);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Command>());
    rclcpp::shutdown();
    return 0;
}