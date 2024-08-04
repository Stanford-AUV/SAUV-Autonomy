#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath> // for std::isinf

#include "PDD_Include.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"


/***
 * 
 * NOTE: you WILL need to add permissions to access the USB port! 
 * > sudo usermod -a -G dialout $(whoami)
 * 
 * ***/

#define DVL_PORT "/dev/ttyUSB_DVL" // see udev rules at /etc/udev/rules.d/99-usb-serial.rules
#define BUFFER_SIZE 256
#define PUBLISH_TIME_MS 1
#define ELIMINATE_INF true

bool firstEnsemble = false;

class DvlPublisher : public rclcpp::Node
{
  public:
    DvlPublisher() : Node("dvl_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/dvl/twist", 10);
      timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
            std::bind(&DvlPublisher::timer_callback, this));
      init_port();
      init_dvl();
      init_decoder();

      RCLCPP_INFO(this->get_logger(), "Running DVL measurements");
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

      tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input, no echo, no signals

      // Setting Input Modes (c_iflag)
      tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
      tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

      // Setting Output Modes (c_oflag)
      tty.c_oflag &= ~OPOST; // Raw output
      tcsetattr(fd_, TCSANOW, &tty);
    }

    void init_dvl()
    {
        const char *config_commands[] = {
            // "CB811",        // 115200 baud, no parity, 1 stop bit
            // "#PD0",         // standard DVL format, machine readable
            // "#CT1",         // turnkey operation (within 10s)
            // "TE00:00:00.00", // ping time is as fast as possible
            // "BX00120",       // ensure max ping range (12m)
            // "EA+04500",      // heading alignment of the DVL
            // "ES00",           // set salinity to 0 PPM
            // "CK",            // save configuration
            "CS",           // begin reading
        };
        const size_t num_commands = sizeof(config_commands) / sizeof(config_commands[0]);

         // send a serial break of 0.3 seconds
        if (tcsendbreak(fd_, 3) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send serial break");
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial break sent successfully");
        }

        for (size_t i = 0; i < num_commands; ++i) {
            int bytes_written = write(fd_, config_commands[i], strlen(config_commands[i]));
            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send DVL configuration command %zu", i + 1);
            } else {
                RCLCPP_INFO(this->get_logger(), "DVL Configuration command %zu sent successfully", i + 1);
            }
            usleep(100000); // Optional: add a small delay between commands (100ms)
        }
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
        // std::stringstream ss;
        // for (int i = 0; i < data; ++i) {
        //     ss << std::hex << std::setfill('0') << std::setw(2) << (int)buffer[i] << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "DATA: %s", ss.str().c_str());

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
            RCLCPP_INFO(this->get_logger(), "Ensemble found!");

            double velArray[FOUR_BEAMS];
            tdym::PDD_GetVesselVelocities(&ens, velArray);
            if (ELIMINATE_INF){
              if(std::isinf(velArray[0]) || std::isinf(velArray[1]) || std::isinf(velArray[2])){
                continue; //skip if any are infinity!
              }
            }

            geometry_msgs::msg::TwistWithCovarianceStamped velMessage; // NOTE: not sure what the order or meaning of the four means are!
            velMessage.header.frame_id = "dvl_link";
            velMessage.header.stamp = this->now();
            velMessage.twist.twist.linear.x = velArray[1];
            velMessage.twist.twist.linear.y = -velArray[0];
            velMessage.twist.twist.linear.z = velArray[2];

            publisher_->publish(velMessage);
            if (firstEnsemble == false){
              RCLCPP_INFO(this->get_logger(), "DVL node is publishing succesfully");
              firstEnsemble = true; 
            }
          }
        } while (found > 0);
      }
      
    }

  //Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
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