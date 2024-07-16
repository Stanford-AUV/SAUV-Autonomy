// MIT License
//
// Copyright (c) 2021 Yuming Meng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef NTRIPLIB_NTRIP_CLIENT_H_
#define NTRIPLIB_NTRIP_CLIENT_H_

#include <atomic>
#include <string>
#include <thread> // NOLINT.
#include <functional>

#include "thread_raii.h"

#include <netdb.h>
#include <sys/types.h>

namespace libntrip
{

    using ClientCallback = std::function<void(char const *_buffer, int _size)>;

    class NtripClient
    {
    public:
        NtripClient() = default;
        NtripClient(NtripClient const &) = delete;
        NtripClient(NtripClient &&) = delete;
        NtripClient &operator=(NtripClient const &) = delete;
        NtripClient &operator=(NtripClient &&) = delete;
        NtripClient(std::string const &ip, int port,
                    std::string const &user, std::string const &passwd,
                    std::string const &mountpoint) : server_ip_(ip), server_port_(port),
                                                     user_(user), passwd_(passwd),
                                                     mountpoint_(mountpoint) {}
        ~NtripClient() { Stop(); }

        void Init(std::string const &ip, int port,
                  std::string const &user, std::string const &passwd,
                  std::string const &mountpoint)
        {
            server_ip_ = ip;
            server_port_ = port;
            user_ = user;
            passwd_ = passwd;
            mountpoint_ = mountpoint;
        }
        // Update the GGA sentence for the server
        void set_gga_buffer(std::string const &gga_buffer)
        {
            gga_buffer_ = gga_buffer;
            gga_is_update_.store(true);
            //print the gga_buffer_
            //printf("ntrip_client.h, line 70:gga_buffer_: %s\n", gga_buffer_.c_str());
        }
        
        // // set fixed location 
        // void set_location(double latitude, double longitude, double height)
        // {
        //     latitude_ = latitude;
        //     longitude_ = longitude;
        //     height_ = height;
        // }


        // set the time interval to report to Ntrip to server, unit:sec
        void set_report_interval(int intv)
        {
            report_interval_ = intv;
        }

        // Set the callback when the RTCM data is received.
        void OnReceived(const ClientCallback &callback) { callback_ = callback; }
        bool Run(void);
        void Stop(void);
        bool service_is_running(void) const
        {
            return service_is_running_.load();
        }
        // Add a new public method to set the flag
        void SetGnssDataReceived(bool received)
        {
            gnss_data_received_ = received;
        }

    private:
        // Thread handler.
        void ThreadHandler(void);

        std::atomic_bool service_is_running_ = {false};
        std::atomic_bool gga_is_update_ = {false}; // update GGA flag
        int report_interval_;                      // GGA data report interval
        double latitude_ = 0.0;                    
        double longitude_ = 0.0;                   
        double height_ = 0.0;                     
        std::string server_ip_;
        int server_port_ = -1;
        std::string user_;
        std::string passwd_;
        std::string mountpoint_;
        std::string gga_buffer_;
        int socket_fd_ = -1;
        Thread thread_;
        ClientCallback callback_ = [](char const *, int) -> void {};
        //check if gnss is received
        bool gnss_data_received_ = false;
        bool resolve_hostname(const std::string& hostname, sockaddr_in& addr);
    };

} // namespace libntrip

#endif // NTRIPLIB_NTRIP_CLIENT_H_
