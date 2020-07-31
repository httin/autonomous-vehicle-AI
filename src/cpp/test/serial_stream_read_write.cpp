/**
 *  @example serial_stream_read_write.cpp
 */

#include <libserial/SerialStream.h>

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <unistd.h>

constexpr const char* const USB_SERIAL_PORT = "/dev/ttyUSB2" ;
constexpr const char* const IMU_SERIAL_PORT = "/dev/ttyUSB1" ;

using namespace LibSerial;
using namespace std;
/**
 * @brief This example demonstrates multiple methods to read and write
 *        serial stream data utilizing std::iostream functionality.
 */
void config_serial(SerialStream& serial, BaudRate baud, CharacterSize size, FlowControl flow, Parity parity, StopBits bits)
{
    serial.SetBaudRate(baud);
    serial.SetCharacterSize(size);
    serial.SetFlowControl(flow);
    serial.SetParity(parity);
    serial.SetStopBits(bits);
}

void *imu_thread(void *thread_data)
{ 
    SerialStream *serial_imu = (SerialStream *)thread_data;
    std::string imu_read = "";
    cout << "[imu_thread] started\n";
    while (std::getline(*serial_imu, imu_read))
    {
        cout << imu_read;
    }
    pthread_exit(NULL);
}

int main()
{
    pthread_t imu_thread_id;
    int rc;
    // Instantiate two SerialStream objects.
    SerialStream serial_stream_usb ;
    SerialStream serial_stream_imu ;

    try
    {
        // Open the Serial Ports at the desired hardware devices.
        serial_stream_usb.Open(USB_SERIAL_PORT) ;
        serial_stream_imu.Open(IMU_SERIAL_PORT) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }

    config_serial(serial_stream_usb, BaudRate::BAUD_115200, CharacterSize::CHAR_SIZE_8,
         FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_NONE, StopBits::STOP_BITS_1);

    config_serial(serial_stream_imu, BaudRate::BAUD_921600, CharacterSize::CHAR_SIZE_8,
         FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_NONE, StopBits::STOP_BITS_1);

    if ( (rc = pthread_create(&imu_thread_id, NULL, imu_thread, (void *)&serial_stream_imu)) != 0)
    {
        cout << "Error:unable to create thread," << rc << endl;
    }
    else 
    {
        cout << "[main thread] created imu thread, thread id: " << imu_thread_id << endl;
    }

    // Variables to store outgoing and incoming data.
    std::string write_string_1 = "\"Do what you can, with what you have, where you are.\" - Theodore Roosevelt" ;
    std::string write_string_2 = "\"Simplicity is prerequisite for reliability.\" - Edsger W. Dijkstra" ;

    std::string read_string_1 = "" ;
    std::string read_string_2 = "" ;

    // Print to the terminal what will take place next.
    std::cout << "\nUsing write() and read() for a specified number of "
              << "bytes of data:" << std::endl ;

    // Write a specified number of bytes of data.
    serial_stream_usb.write(write_string_1.c_str(), write_string_1.size()) ;
    serial_stream_imu.write(write_string_2.c_str(), write_string_2.size()) ;

    // Wait until the data has actually been transmitted.
    serial_stream_usb.DrainWriteBuffer() ;
    serial_stream_imu.DrainWriteBuffer() ;

    // Char arrays to store incoming data.
    char* read_array_1 = new char[write_string_2.size()] ;
    char* read_array_2 = new char[write_string_1.size()] ;

    // Use inheritted std::istream read() method to read the data.
    serial_stream_usb.read(read_array_1, write_string_2.size()) ;
    serial_stream_imu.read(read_array_2, write_string_1.size()) ;

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_string_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_array_2  << std::endl
              << std::endl ;

    std::cout << "\tSerial Port 2 sent:\t"     << write_string_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_array_1  << std::endl
              << std::endl ;

    // Print to the terminal what will take place next.
    std::cout << "Using the \"<<\" operator and getline() for a line of data:"
              << std::endl ;

    // Write a line at each serial port.
    serial_stream_usb << write_string_1 << std::endl ;
    serial_stream_imu << write_string_2 << std::endl ;

    // Wait until the data has actually been transmitted.
    serial_stream_usb.DrainWriteBuffer() ;
    serial_stream_imu.DrainWriteBuffer() ;

    // Read a line at each serial port.
    std::getline(serial_stream_usb, read_string_1) ;
    std::getline(serial_stream_imu, read_string_2) ;

    // Print to the terminal what was sent and what was received.
    std::cout << "\tSerial Port 1 sent:\t"     << write_string_1 << std::endl
              << "\tSerial Port 2 received:\t" << read_string_2  << std::endl
              << std::endl ;

    std::cout << "\tSerial Port 2 sent:\t"     << write_string_2 << std::endl
              << "\tSerial Port 1 received:\t" << read_string_1  << std::endl
              << std::endl ;

    // Variable to hold user input.
    std::string user_input ;
    user_input.clear() ;

    // Prompt the user for input.
    std::cout << "Type something you would like to send over serial,"
              << " (enter \"Q\" or \"q\" to quit): " << std::flush ;

    while(true)
    {
        // Get input from the user.
        std::getline(std::cin, user_input) ;

        if (user_input == "q" ||
            user_input == "Q" ||
            user_input == "")
        {
            break ;
        }

        // Print to the terminal what will take place next.
        std::cout << "Using the \"<<\" and \">>\" operators to send "
                  << "and receive your data: " << std::endl ;

        // Write the user input to the serial port.
        serial_stream_usb << user_input << std::endl ;

        // Read the data transmitted from the corresponding serial port.
        // Note: The ">>" operator behavior is tricky if whitespace or
        //       nothing is entered by the user!
        serial_stream_imu >> read_string_2 ;

        // Print to the terminal what was sent and what was received.
        std::cout << "\tSerial Port 1 sent:\t"     << user_input   << std::endl
                  << "\tSerial Port 2 received:\t" << read_string_2 << std::endl ;
    }

    // Close the serial ports and end the program.
    serial_stream_usb.Close() ;
    serial_stream_imu.Close() ;

    // Successful program completion.
    std::cout << "The example program successfully completed!" << std::endl ;
    return EXIT_SUCCESS ;
}
