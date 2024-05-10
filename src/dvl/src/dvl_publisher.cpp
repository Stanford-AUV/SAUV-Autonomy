#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "PDD_Include.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

/***
 * 
 * NOTE: you WILL need to add permissions to access the USB port! 
 * > sudo usermod -a -G dialout $(whoami)
 * 
 * ***/

#define DVL_PORT "/dev/ttyUSB0" // this must be found first!!
#define BUFFER_SIZE 10000
#define PUBLISH_TIME_MS 10

class DvlPublisher : public rclcpp::Node
{
  public:
    DvlPublisher() : Node("dvl_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/dvl/velocity", 10);
      timer_ = this->create_wall_timer(
            std::chrono::microseconds(PUBLISH_TIME_MS),
            std::bind(&DvlPublisher::timer_callback, this));
      init_port();
      init_decoder();
      //one more to force PD0 on DVL?
    }

    ~DvlPublisher()
    {
      delete decoder;
      decoder = nullptr;
      delete[] buffer;
      buffer = nullptr;
      close(fd_);
    }

  private:
    void init_port()
    {
        fd_ = open(DVL_PORT, O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            return;
        }

        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);    // Enable the receiver and set local mode
        tty.c_cflag &= ~PARENB;             // No parity bit
        tty.c_cflag &= ~CSTOPB;             // 1 stop bit
        tty.c_cflag &= ~CSIZE;              // Mask character size bits
        tty.c_cflag |= CS8;                 // 8 data bits
        tcsetattr(fd_, TCSANOW, &tty);
    }

    void init_decoder()
    {
      tdym::PDD_SetInvalidValue(INFINITY); //force invalid data to be infinity
      tdym::PDD_InitializeDecoder(decoder); //init decoder
    }

    void timer_callback()
    {
      std::memset(buffer, 0, BUFFER_SIZE);
      int data = read(fd_, buffer, sizeof(buffer));

      if (data > 0) {
        std::stringstream ss;
        for (int i = 0; i < data; ++i) {
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)buffer[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "DATA: %s", ss.str().c_str());

        // std::stringstream ss2;
        // ss2 << std::hex << std::setfill('0') << std::setw(2) << (int)data << " ";
        // RCLCPP_INFO(this->get_logger(), "SIZE: %s", ss2.str().c_str());

        tdym::PDD_AddDecoderData(decoder, buffer, (int)data);
        int found = 0;
        do
        {
          found = tdym::PDD_GetPD0Ensemble(decoder, &ens);
          if (found)
          {
            RCLCPP_INFO(this->get_logger(), "FOUND!!!!!");
            // RCLCPP_INFO(this->get_logger(), "FOUND!!!!!!!!");
            // extract data from ensemble and send to geometry_msgs
            // IGNORE BELOW

            double velArray[FOUR_BEAMS];
            tdym::PDD_GetVesselVelocities(&ens, velArray);
            geometry_msgs::msg::Twist velMessage; // NOTE: not sure what the order or meaning of the four means are!
            velMessage.linear.x = velArray[0];
            velMessage.linear.y = velArray[1];
            velMessage.linear.z = velArray[2];
            publisher_->publish(message);

            timespec time = tdym::PDD_GetTimestamp(&ens)
            std::stringstream ss;
            ss << time.tv_sec;
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            // IGNORE ABOVE
          }
        } while (found > 0);
      }
    }

  //Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  int fd_;
  unsigned char* buffer = new unsigned char[BUFFER_SIZE];
  tdym::PDD_Decoder* decoder = new tdym::PDD_Decoder();
  tdym::PDD_PD0Ensemble ens;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlPublisher>());
  rclcpp::shutdown();
  return 0;
}