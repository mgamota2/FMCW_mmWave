// Read the documentation for the DCA1000EVM:
// https://www.ti.com/lit/ug/spruij4a/spruij4a.pdf?ts=1709266126258&ref_url=https%253A%252F%252Fwww.google.com%252F

#include <map>
#include <string>
#include <sys/socket.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <errno.h>
#include <chrono>
#include <cerrno>
#include <iomanip>
#include <cstdint>
#include <queue>

// These come from the documentation for the DCA1000EVM
#define RESET_FPGA_CMD_CODE                  ((const char*)"")
#define RESET_AR_DEV_CMD_CODE                ((const char*)"")
#define CONFIG_FPGA_GEN_CMD_CODE             ((const char*)"\x5a\xa5\x03\x00\x06\x00\x01\x01\x01\x02\x03\x1e\xaa\xee")
#define CONFIG_EEPROM_CMD_CODE               ((const char*)"")
#define RECORD_START_CMD_CODE                ((const char*)"\x5a\xa5\x05\x00\x00\x00\xaa\xee")
#define RECORD_STOP_CMD_CODE                 ((const char*)"\x5a\xa5\x06\x00\x00\x00\xaa\xee")
#define PLAYBACK_START_CMD_CODE              ((const char*)"")
#define PLAYBACK_STOP_CMD_CODE               ((const char*)"")
#define SYSTEM_CONNECT_CMD_CODE              ((const char*)"\x5a\xa5\x09\x00\x00\x00\xaa\xee")
#define SYSTEM_ERROR_CMD_CODE                ((const char*)"\x5a\xa5\x0a\x00\x01\x00\xaa\xee")
#define CONFIG_PACKET_DATA_CMD_CODE          ((const char*)"\x5a\xa5\x0b\x00\x06\x00\xc0\x05\xc4\x09\x00\x00\xaa\xee")
#define CONFIG_DATA_MODE_AR_DEV_CMD_CODE     ((const char*)"")
#define INIT_FPGA_PLAYBACK_CMD_CODE          ((const char*)"")
#define READ_FPGA_VERSION_CMD_CODE           ((const char*)"\x5a\xa5\x0e\x00\x00\x00\xaa\xee")

// The ring buffer is used to store the incoming data
class ring_buffer{
    public:
        int64_t max_len;
        int16_t* data;
        std::queue<uint16_t> q;
        int64_t put_idx;
        int64_t frame_size;
        int16_t pop_array;
        int16_t n_frames;
        
        // Should be a multiple of the frame size since it is storing frames
        ring_buffer(int32_t this_max_len, int32_t this_frame_size){
            max_len = this_max_len;
            data = new int16_t[max_len];
            // std::cout<<"Able to create data array"<<std::endl;
            // std::cout<<*data<<"Able to access data array"<<std::endl;

            frame_size = this_frame_size;
            pop_array = -1;
            put_idx=0;

            if (max_len % frame_size == 0){
                n_frames = int16_t(max_len / frame_size);
            }
            else{
                std::cout<<"Must be multiple of frame size"<<std::endl;
                return;

            }
        }

        ~ring_buffer(){
            delete[] data;
        }

        int16_t find_pops(int64_t  old_put_idx,
                    int64_t  new_put_idx,
                    int64_t  frame_size)
        {
            int32_t old_frame_idx = old_put_idx / frame_size;
            int32_t new_frame_idx = new_put_idx / frame_size;
            return(old_frame_idx == new_frame_idx) ? -1 : old_frame_idx;
        }

        // If we receive an incomplete frame, add zeros
        void add_zeros(int64_t num_zeros,
                    int16_t* buffer,
                    int64_t buffer_len,
                    int64_t* put_idx,
                    int64_t frame_size,
                    int16_t* pop_frame_idx)
        {
            int32_t new_put_idx = *put_idx;
            int32_t to_end_of_frame = frame_size - (*put_idx % frame_size); // number of cell to the end of current frame
            if (num_zeros < to_end_of_frame) { // if does not reach the end of the frame
                // std::cout<<"Seg fault in pad_msg if"<<std::endl;
                // std::cout<<*put_idx<<"Put index"<<std::endl;
                // std::cout<<*put_idx<<std::endl;
                // std::cout<<"Trying to access buffer"<<std::endl;
                // std::cout<<*buffer<<"Buffer"<<std::endl;
                memset(buffer + *put_idx, 0, num_zeros * sizeof(buffer[0])); // set zeros accordingly
                new_put_idx += num_zeros; // move pointer
            }

            else {	// if overflow
                // std::cout<<"Seg fault in pad_msg else"<<std::endl;
                // std::cout<<*put_idx<<"Put index"<<std::endl;
                // std::cout<<*put_idx<<std::endl;
                // std::cout<<"Trying to access buffer"<<std::endl;
                // std::cout<<*buffer<<"Buffer"<<std::endl;
                memset(buffer + *put_idx, 0, to_end_of_frame * sizeof(buffer[0])); // set all unfinished cell of frame to 0
                num_zeros -= to_end_of_frame; // substract those 0 filled
                new_put_idx += to_end_of_frame; // move to start of the next frame
                if (new_put_idx >= buffer_len) new_put_idx -= buffer_len; // loop if necessary
                num_zeros %= frame_size; // the rest of the needed zeros (skip 0-filled frames)
                memset(buffer + new_put_idx, 0, num_zeros * sizeof(buffer[0])); // fill zeros to destination idx
                new_put_idx += num_zeros; // move pointer
            }
            *pop_frame_idx = find_pops(*put_idx, new_put_idx, frame_size);
            *put_idx = new_put_idx;
            // add_to_queue();

        }

        // Put the message in the buffer
        void add_msg(int16_t* msg,
                    int16_t msg_len,
                    int16_t* buffer,
                    int64_t buffer_len,
                    int64_t* put_idx,
                    int64_t frame_size,
                    int16_t* pop_frame_idx)
        {
            int32_t new_put_idx = *put_idx;
            if (*put_idx + msg_len <= buffer_len) //did not loop around to beginning
            {
                // std::cout<<"Seg fault in add_msg if"<<std::endl;
                // std::cout<<*put_idx<<"Put index"<<std::endl;
                // std::cout<<*put_idx<<std::endl;
                // std::cout<<"Trying to access buffer"<<std::endl;
                // std::cout<<*buffer<<"Buffer"<<std::endl;
                memcpy(buffer + *put_idx, msg, msg_len*sizeof(msg[0]));
                new_put_idx = *put_idx + msg_len; //new location of put idx
                if (new_put_idx >= buffer_len) new_put_idx -= buffer_len;
            }
            else // did loop around to beginning
            {

                memcpy(buffer + *put_idx, msg, (buffer_len - *put_idx)*sizeof(msg[0]));
                memcpy(buffer, msg + (buffer_len - *put_idx), (msg_len - buffer_len + *put_idx)*sizeof(msg[0]));
                new_put_idx = msg_len - buffer_len + *put_idx;
            }

            *pop_frame_idx = find_pops(*put_idx, new_put_idx, frame_size);
            *put_idx = new_put_idx;

        }

        // This function is actually called, this one adds the frame to the ring buffer, but if it's incomplete, pad zeros at the end
        void pad_and_add_msg(int64_t seq_c,
                int64_t seq_n,
                int16_t* msg,
                int16_t msg_len,
                int16_t* buffer,
                int64_t buffer_len,
                int64_t* put_idx,
                int64_t frame_size,
                int16_t* pop_frame_idx){
            //determine if zeros needed
            int64_t num_zeros = (seq_n - seq_c - 1) * 728;
            // std::cout<<"Put index in pad and add msg"<<*put_idx<<std::endl;

            // printf(stderr, "INFO: current sequence number %ld, received %ld\n", seq_c, seq_n);
            if(num_zeros > 0){
                fprintf(stderr, "WARN: Padding %ld zeros\n", num_zeros);
                add_zeros(num_zeros, buffer, buffer_len, put_idx, frame_size, pop_frame_idx);
            }
            add_msg(msg, msg_len, buffer, buffer_len, put_idx, frame_size, pop_frame_idx);
        }

    
    private:
        
        
};

// Main class for the mmWave radar system
class mmWave_Sensor{
    public: 
        ring_buffer* data_array_ptr;
        int16_t* msg;
        // Requires a config file
        mmWave_Sensor(std::string config) {
            config_name = config;
            std::cout<<"GET CONFIG DATA"<<std::endl;
            std::string base_path="/home/radar/dev_ws/src/cpp_mmwavec/src/configs/";
            std::string dot_cfg=".cfg";
            std::string full_path = base_path+config+dot_cfg;
            std::ifstream frame_data(full_path);

            std::string cmd;
            int chirps =0;
            int lanes =0;
            int samples =0;
            if (!frame_data.is_open()) {
                std::cerr << "Error opening file: " << full_path << std::endl;
                return;
            }
            // Parse the config file to get the number of RX/TX antennas, ADC samples per frame, loops per fram
            while (std::getline(frame_data, cmd)) {
                std::istringstream ss(cmd);
                std::string token;
                ss >> token;  // Get the first word in the line (e.g., "profileCfg" or "frameCfg")

                if (token == "profileCfg") {
                    int count = 0;
                    while (ss >> token) {
                        if (count == 9) {  // The 9th token is the 256 value we want
                            samples = std::stoi(token);
                        }
                        count++;
                    }
                } 
                else if (token == "frameCfg") {
                    int count = 0;
                    while (ss >> token) {
                        if (count == 2) {  // The 3rd token is the 128 value we want
                            chirps = std::stoi(token);
                        }
                        count++;
                    }
                }
                else if (token == "channelCfg") {
                    int count = 0;
                    
                    while (ss >> token) {
                        if (count == 0) {  // The RX lanes are one-hot encoded
                            uint8_t value;
                            value = std::stoi(token);
                            // std::cout<<"value"<<value<<std::endl;
                            if (value==15){
                                lanes=4;
                            }
                            else if (value==7){
                                lanes=3;
                            }
                            else if(value==3){
                                lanes=2;
                            }
                            else if (value==1){
                                lanes=1;
                            }
                            else {
                                lanes=4; //No reason to use less than 4
                            }
                        }
                    }
                }
            }
            // std::cout<<"chirps: "<<chirps<<std::endl;
            // std::cout<<"samples: "<<samples<<std::endl;
            // std::cout<<"lanes: "<<lanes<<std::endl;


            frame_data.close();
            // After we have parsed the config file, calcluate the frame length, use to create a ring buffer
            frame_len = 2*samples*lanes*chirps;

            std::cout << "frame length:"<<frame_len <<std::endl;
            data_array_ptr = new ring_buffer(int32_t(2*frame_len), int32_t(frame_len));


            // The DCA1000EVM is connected via a socket (physically a Cat5 cable)
            data_socket=socket(AF_INET, SOCK_DGRAM, 0);
            sockaddr_in address;
            memset(&address, '\0', sizeof(address)); 
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = inet_addr("192.168.33.30"); // This is the DCA1000EVM IP
            address.sin_port=htons(4098);
            
            if (bind(data_socket, (struct sockaddr*)&address, sizeof(address))<0){
                std::cerr << "Error binding socket" << std::endl;
                close(data_socket);
                return;
            }
            dca_socket=socket(AF_INET, SOCK_DGRAM, 0);
            sockaddr_in dca_address;
            memset(&dca_address, '\0', sizeof(dca_address)); 
            dca_address.sin_family = AF_INET;
            dca_address.sin_addr.s_addr = inet_addr("192.168.33.30"); // This is the DCA1000EVM IP
            dca_address.sin_port=htons(4096);
            
            if (bind(dca_socket, (struct sockaddr*)&dca_address, sizeof(dca_address))<0){
                std::cerr << "Error binding socket" << std::endl;
                close(dca_socket);
                return;
            }
            dca_socket_open=true;

            dca_client_addr.sin_family = AF_INET;
            dca_client_addr.sin_addr.s_addr = inet_addr("192.168.33.180"); // This is the DCA1000EVM FPGA IP
            dca_client_addr.sin_port = htons(4096);
            // We have now configured the socket for the DCA1000EVM

            // The IWR1443BOOST radar board is connected via serial (physically a micro USB cable)
            iwr_serial = open(iwr_cmd_tty, O_RDWR);
            if (iwr_serial == -1) {
                std::cerr << "Failed to open the serial port" << std::endl;
                return;
            }

            // Configure serial port settings
            struct termios tty;
            memset(&tty, 0, sizeof(tty));
            if (tcgetattr(iwr_serial, &tty) != 0) {
                std::cerr << "Error configuring serial port" << std::endl;
                close(iwr_serial);
                return;
            }
            tty.c_cflag &= ~PARENB; // Disable parity
            tty.c_cflag &= ~CSTOPB; // 1 stop bit
            tty.c_cflag |= CS8;     // 8 data bits
            // Set baud rate (example: 115200)
            cfsetospeed(&tty, B115200);
            cfsetispeed(&tty, B115200);

            // Apply settings
            if (tcsetattr(iwr_serial, TCSANOW, &tty) != 0) {
                std::cerr << "Error applying serial port settings" << std::endl;
                close(iwr_serial);
                return;
            }

            // Flush the serial port (both input and output buffers)
            for (int i=5; i>0;i--){
                // write(iwr_serial, &newline, sizeof(newline));
                int err = tcflush(iwr_serial, TCIOFLUSH);
                if (err != 0) {
                    std::cerr << "Error flushing serial port: "<<errno << std::endl;
                    return;
                }
            }
            std::cout<<"Flushed serial port"<<std::endl;
            
            serial_open = true;
            // We have now configured the serial connection to the IWR1443BOOST

        }
        ~mmWave_Sensor(){
            delete data_array_ptr;
        }

        void close_comms(){
            std::cout<<"Closing sockets and serial connections"<<std::endl;
            close(data_socket);
            close(dca_socket);
            close(iwr_serial);
            std::cout<<"Closed"<<std::endl;
        }

        // Generic socket read for DCA1000EVM configuration commands
        void collect_response() {
            try {
                char buffer[2048];
                socklen_t serverAddrLen = sizeof(dca_cmd_addr);
                ssize_t bytesReceived = recvfrom(dca_socket, buffer, sizeof(buffer), 0, (struct sockaddr*)&dca_cmd_addr, &serverAddrLen);
                if (bytesReceived < 0) {
                    throw std::runtime_error("Socket receive failed");
                }

                // Assuming the status is at bytes 4 to 5 (inclusive) in the received message
                // std::memcpy(&status, &buffer[4], sizeof(status));
                // status = ntohs(status); // Convert from network byte order to host byte order
                //For the FPGA version command, 898 is the "correct" response
                
            } catch (const std::exception& e) {
                std::cerr << "Error: " << e.what() << std::endl;
            }
            
            
            return; 
        }

        // Arming the DCA1000EVM to begin expecting data
        void arm_dca(){

            std::cout<<"ARM DCA"<<std::endl;

            int err = sendto(dca_socket, RECORD_START_CMD_CODE, sizeof(RECORD_START_CMD_CODE), 0, (struct sockaddr*)&dca_client_addr, sizeof(dca_client_addr));
            std::cout<<"Sent arm cmd to DCA, sent " << err<< " bytes"<<std::endl;
            collect_response();
            std::cout<<"Success"<<std::endl;
        }

        // This configures both the DCA1000EVM (connected via socket) and the IWR1443BOOST (connected via serial)
        int setupDCA_and_cfgIWR(){
            
            // Set up the DCA1000EVM board (setting the data packet format, etc.)
            if (!dca_socket_open||!serial_open){
                std::cout<<"Serial/socket error (maybe try sudo chmod 666 /dev/ttyACM0)"<<std::endl;
                return -1;
            }

            sockaddr_in dca_client_addr;
            dca_client_addr.sin_family = AF_INET;
            dca_client_addr.sin_addr.s_addr = inet_addr("192.168.33.180");
            dca_client_addr.sin_port = htons(4096);

            std::cout<<"SET UP DCA"<<std::endl;

            int err = sendto(dca_socket, SYSTEM_CONNECT_CMD_CODE, sizeof(SYSTEM_CONNECT_CMD_CODE), 0, (struct sockaddr*)&dca_client_addr, sizeof(dca_client_addr));
            std::cout<<"Sent connect cmd to DCA, sent " << err<< " bytes"<<std::endl;
            collect_response();
            std::cout<<"Collected"<<std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms delay after sending command

            err = sendto(dca_socket, READ_FPGA_VERSION_CMD_CODE, sizeof(READ_FPGA_VERSION_CMD_CODE), 0, (struct sockaddr*)&dca_client_addr, sizeof(dca_client_addr));
            std::cout<<"Sent read fpga version cmd to DCA, sent " << err<< " bytes"<<std::endl;
            collect_response();
            std::cout<<"Collected"<<std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms delay after sending command

            err = sendto(dca_socket, CONFIG_FPGA_GEN_CMD_CODE, sizeof("\x5a\xa5\x03\x00\x06\x00\x01\x01\x01\x02\x03\x1e\xaa\xee"), 0, (struct sockaddr*)&dca_client_addr, sizeof(dca_client_addr));
            std::cout<<"Sent config fpga cmd to DCA, sent " << err<< " bytes"<<std::endl;
            collect_response();
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms delay after sending command

            err = sendto(dca_socket, CONFIG_PACKET_DATA_CMD_CODE, sizeof(CONFIG_PACKET_DATA_CMD_CODE), 0, (struct sockaddr*)&dca_client_addr, sizeof(dca_client_addr));
            std::cout<<"Sent config packet data cmd to DCA, sent " << err<< " bytes"<<std::endl;
            collect_response();
            std::cout<<"Collected"<<std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms delay after sending command

            // IWR1443BOOST configuration (this is from the config file)
            std::cout<<"CONFIGURE IWR"<<std::endl;
            std::string base_path="/home/radar/dev_ws/src/cpp_mmwavec/src/configs/";
            std::string dot_cfg=".cfg";
            std::string full_path = base_path+config_name+dot_cfg;
            std::ifstream config(full_path);

            // Check if the file is opened successfully
            if (!config.is_open()) {
                std::cerr << "Error: Unable to open the file." << std::endl;
                return -1;
            }
            
            char newline = '\r';
            
            // Send the config line by line to the IWR1443BOOST to configure the chirps and frames
            std::string cmd;
            while(getline(config, cmd)){
                for (size_t j = 0; j < size(cmd); j++) {  // Use j as the loop counter
                    int err = write(iwr_serial, &cmd[j], 1); // Write each character individually
                    if (err == -1) {
                        std::cerr << "Error writing to serial port" << std::endl;
                        close(iwr_serial);
                        return -1;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms delay between characters
                }
                write(iwr_serial, &newline, 1); // Write newline character after each command

                std::this_thread::sleep_for(std::chrono::milliseconds(110)); // 100 ms delay between lines

                char response[300]; // Assuming max response size is 6 characters + null terminator
                ssize_t num_bytes = read(iwr_serial, response, sizeof(response));
                //std::cout<<"num bytes: "<<num_bytes<<std::endl;
                if (num_bytes == -1) {
                    std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
                    close(iwr_serial);
                    return -1;
                }
                while (num_bytes<1){
                    num_bytes = read(iwr_serial, response, 6);
                }
                std::cout << "LVDS Stream:/>" << cmd << std::endl;
                std::cout<< '\r' << std::endl;
                std::cout << "Response: " << response << std::endl;
                
            }
            // Close the file
            config.close();   
            return 1; 
        }  

        // This sends a start/stop capture command to the IWR1443 based on the toggle argument (0=stop, 1=start)
        void toggle_capture(uint8_t toggle){
            if (capture_started==toggle){
                return;
            }
            capture_started=toggle;
            char newline = '\r';
            const char* cmd=iwr_rec_cmd[toggle];
            for (size_t j = 0; j < strlen(cmd); j++) {  // Use j as the loop counter
                int err = write(iwr_serial, &cmd[j], 1); // Write each character individually
                if (err == -1) {
                    std::cerr << "Error writing to serial port" << std::endl;
                    close(iwr_serial);
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 10 ms delay between characters
            }
            write(iwr_serial, &newline, 1); // Write newline character after each command

            std::this_thread::sleep_for(std::chrono::milliseconds(110)); // 100 ms delay between lines

            char response[300]; // Assuming max response size is 6 characters + null terminator
            ssize_t num_bytes = read(iwr_serial, response, sizeof(response));
            if (num_bytes == -1) {
                std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
                close(iwr_serial);
                return ;
            }
            while (num_bytes<1){
                num_bytes = read(iwr_serial, response, 6);
            }
            std::cout << "LVDS Stream:/>" << cmd << std::endl;
            std::cout<< '\r' << std::endl;
            std::cout << "Response: " << response << std::endl;

            if (strcmp(cmd, "sensorStop")==0){
                int8_t err = sendto(dca_socket, RECORD_STOP_CMD_CODE, sizeof(RECORD_STOP_CMD_CODE), 0, (struct sockaddr*)&dca_client_addr, sizeof(dca_client_addr));
                std::cout<<"Sent STOP cmd to DCA, sent " << err<< " bytes"<<std::endl;
                if (err<0){
                    std::cout<<"Error: "<< errno <<std::endl;
                }
                collect_response();
            }
            

        }

        // Packet structure is raw mode data format (see documentation)
        void collect_data(){
            uint16_t buf[4096];
            uint16_t incoming = recv(data_socket, buf, sizeof(buf), 0);
            
            // Separate the message from the preamble
            size_t data_len = incoming - 5;
            int64_t seqn_new;
            std::memcpy(&seqn_new, buf, 4*sizeof(uint8_t));
            // Dynamic memory allocation for msg based on data_len
            msg = new int16_t[data_len];

            // Calculate the number of elements to copy
            uint32_t num_elements = (data_len - 5) / sizeof(int16_t);

            // Copy data from buf to msg
            std::memcpy(msg, buf + 5, num_elements * sizeof(int16_t));           
            
            // This puts the current message into the ring buffer
            data_array_ptr->pad_and_add_msg(this->seqc, seqn_new, msg, num_elements, \
            data_array_ptr->data, data_array_ptr->max_len, \
            &(data_array_ptr->put_idx), data_array_ptr->frame_size,\
            &(data_array_ptr->pop_array));
            
            this->seqc = seqn_new;
            delete[] msg;

        }
        // Helper function to see if we are currently sending chirps
        uint8_t is_capture_started(){
            return capture_started;
        }

    private:
        std::string config_name; //Config file name
        const char* iwr_rec_cmd[2]={"sensorStop", "sensorStart"};

        const char* dca_cmd_addr = "192.168.33.180"; // This is the DCA1000EVM FPGA IP
        uint16_t dca_cmd_addr_port  =  4096;
        int dca_socket;
        int data_socket;
        sockaddr_in dca_client_addr;

        uint64_t seqc=0;
        uint64_t frame_len;
        
        const char* iwr_cmd_tty = "/dev/ttyACM0";
        int iwr_serial;

        bool dca_socket_open = false;
        bool data_socket_open = false;
        bool serial_open = false;

        uint8_t capture_started = 0;

};