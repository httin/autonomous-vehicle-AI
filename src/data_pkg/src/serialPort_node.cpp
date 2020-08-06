#include "serial_helper.h"

// Specify a timeout value (in milliseconds). If timeout is zero, block infinitely
static size_t g_timeout_milliseconds = 20;
static size_t g_imu_buffer_size;

static my_serials_t my_serials;


void *imu_thread(void *thread_data)
{ 
    my_serials_t *my_serials = (my_serials_t *)thread_data;
    
    cout << "[INFO] imu_thread started\n";
    while (true)
    {
        std::string imuWriteData;
        if(g_imu_buffer_size == 0)
        {
            my_serials->imu_port->ReadLine(my_serials->imuReadData, 0x0D, g_timeout_milliseconds);
            g_imu_buffer_size = my_serials->imuReadData.length();
        }
        else 
        {
            my_serials->imu_port->Read(my_serials->imuReadData, g_imu_buffer_size, g_timeout_milliseconds);
        }
        /* Copy original read data to another for writing */
        imuWriteData = my_serials->imuReadData; 
        cout << my_serials->imuReadData;
        /* Acquired lock to write data to USB port */
        pthread_mutex_lock(&my_serials->usb_lockWrite);
        my_serials->usb_port->Write(imuWriteData);
        pthread_mutex_unlock(&my_serials->usb_lockWrite);
    }
    pthread_exit(NULL);
}

void *avoidObstacle_thread(void *thread_data)
{
    cout << "[INFO] avoidObstacle_thread started\n";
    while (true)
    {
        /* TODO: write PC DATA over USB */
        pthread_mutex_lock(&my_serials.usb_lockWrite);

        pthread_mutex_unlock(&my_serials.usb_lockWrite);
    }
    pthread_exit(NULL);
}

void avoidObstacle_callback(const std_msgs::Float64MultiArray &msg)
{
    std::vector<double> myData = msg.data;
    std::string pcDataWrite;

    if ( myData.size() == 1) //
    {
        std::string mess = "PCDAT,0,";
        pcDataWrite = formatMessage(mess);
    }
    else
    {
        std::string velo = boost::lexical_cast<std::string>(myData[2]);
        std::string delta_angle = boost::lexical_cast<std::string>(myData[3]);
        std::string mess = "PCDAT,1," + velo + "," + delta_angle + ",";

        pcDataWrite = formatMessage(mess);
    }
    my_serials.log_ofs << INFO(avoidObstacle_callback, pcDataWrite);
    pthread_mutex_lock(&my_serials.usb_lockWrite);
    my_serials.usb_port->Write(pcDataWrite);
    pthread_mutex_unlock(&my_serials.usb_lockWrite);
}
/**
 * @brief This example demonstrates multiple methods to read and write
 *        serial stream data.
 */
int main(int argc, char **argv)
{
    int rc;
    std::vector<std::string> usbReadDataVector;
    std::vector<coordinate_ns::mapData_t> mapCoordinatesList;
    // System starting time
    int64_t startTime_s = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();

    // Announce this program to the ROS master as a "node" called "serialPort_node"
    ros::init(argc, argv, "serialPort_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("data_topic", 10);
    ros::Subscriber sub = nh.subscribe("data_topic", 10, avoidObstacle_callback);

    std::stringstream    log_ss;
    log_ss << startTime_s << ".log";

    // Instantiate my_serials object.
    my_serials.usb_port = new SerialPort();
    my_serials.imu_port = new SerialPort();
    my_serials.imuReadData = "";
    my_serials.usbReadData = "";
    if ( pthread_mutex_init(&my_serials.usb_lockWrite, NULL) != 0 ||
         pthread_mutex_init(&my_serials.usb_lockRead, NULL) != 0 ) 
    { 
        cout << "mutex initiation has failed\n"; 
        return 1; 
    } 
    my_serials.log_ofs.open(log_ss.str()); 

    try
    {
        // Open the Serial Ports at the desired hardware devices.
        my_serials.usb_port->Open(USB_SERIAL_PORT) ;
        my_serials.imu_port->Open(IMU_SERIAL_PORT) ;
    }
    catch (const OpenFailed&)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl ;
        return EXIT_FAILURE ;
    }

    config_serial(my_serials.usb_port, BaudRate::BAUD_115200, CharacterSize::CHAR_SIZE_8,
         FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_NONE, StopBits::STOP_BITS_1);

    config_serial(my_serials.imu_port, BaudRate::BAUD_921600, CharacterSize::CHAR_SIZE_8,
         FlowControl::FLOW_CONTROL_NONE, Parity::PARITY_NONE, StopBits::STOP_BITS_1);

    while (ros::ok())
    {
        my_serials.usb_port->ReadLine(my_serials.usbReadData);
        usbReadDataVector = split(my_serials.usbReadData, ',');
        my_serials.log_ofs << INFO(main, my_serials.usbReadData);

        if(usbReadDataVector[0].compare("$VPLAN") == 0)
        {
            if(usbReadDataVector[1].compare("STOP") == 0)
            {
                std::stringstream    ss;
                std::ofstream        map_ofs("map_out");
                nh::json j = mapCoordinatesList;
                
                ss << std::setw(4) << j;
                if (map_ofs.is_open())
                {
                    map_ofs << ss.str() << std::endl;
                    map_ofs.close();
                }
                else
                    cout << "[Error] unable to open file map_out" << endl;

                if ( (rc = pthread_create(&my_serials.imu_thread_id, NULL, imu_thread, (void *)&my_serials)) != 0)
                {
                    cout << "Error:unable to create thread, return " << rc << endl;
                    return rc;
                }
                else 
                    cout << "[main thread] created imu thread, thread id: " << my_serials.imu_thread_id << endl;
            }
            else if(usbReadDataVector[1].compare("SPLINE") == 0)
            {
                /* TODO: Handle vehicle message "$VPLAN,SPLINE,<data>,CC\r\n" */
            }
            else 
            {
                size_t index = std::stod(usbReadDataVector[1]);
                double x = std::stod(usbReadDataVector[2]);
                double y = std::stod(usbReadDataVector[3]);
                mapCoordinatesList.push_back( coordinate_ns::mapData{index, x, y} );
            }
        }
        else if(usbReadDataVector[0].compare("$VDATA"))
        {
            std_msgs::Float64MultiArray avoidObstacleMsg;
            avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[1])); // current position X
            avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[2])); // current position y
            avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[3])); // heading angle
            avoidObstacleMsg.data.push_back(std::stod(usbReadDataVector[4])); // point's index
            pub.publish(avoidObstacleMsg);
        }
        else 
        {
            cout << "unknown type: " << usbReadDataVector[0] << endl;
            break;
        }

        ros::spinOnce();
    }

    my_serials.log_ofs.close();
    // Join thread to terminate
    pthread_join(my_serials.imu_thread_id, NULL); 
    pthread_mutex_destroy( &(my_serials.usb_lockWrite) );
    pthread_mutex_destroy( &(my_serials.usb_lockRead) );
    // Close the serial ports and end the program.
    my_serials.usb_port->Close() ;
    my_serials.imu_port->Close() ;
    delete my_serials.usb_port;
    delete my_serials.imu_port;

    // Successful program completion.
    std::cout << "The example program successfully completed!" << std::endl ;
    return EXIT_SUCCESS ;
}
