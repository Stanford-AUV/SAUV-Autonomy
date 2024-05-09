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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

#define DVL_PORT "/dev/ttyUSB0" // this must be found first!!
#define BUFFER_SIZE 256
#define PUBLISH_TIME_MS 100

class DvlPublisher : public rclcpp::Node
{
  public:
    DvlPublisher() : Node("dvl_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/dvl/velocity", 10);
      timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
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
        fd_ = open(DVL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
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
      int data = read(fd_, buffer, sizeof(buffer));

      if (data > 0) {
        tdym::PDD_AddDecoderData(decoder, buffer, data);
        int found = 0;
        do
        {
          found = tdym::PDD_GetPD0Ensemble(decoder, &ens);
          if (found)
          {
            // extract data from ensemble and send to geometry_msgs
            // IGNORE BELOW
            geometry_msgs::msg::Twist message;
            message.linear.x = 3.1415;
            message.angular.z = -3.1415;
            publisher_->publish(message);
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