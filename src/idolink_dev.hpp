#ifndef __IDOLINK_DEV_HPP__
#define __IDOLINK_DEV_HPP__

#include <termios.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <vector>

class IdolinkDev 
{
public:

	//! Range data
	struct RangeData 
	{
		uint64_t sourceId;			// Unique source node ID
		uint64_t destinationId;		// Unique destination node ID
		float range;				// Estimated range in meters
		float rssi;					// Received signal strength in dBm
		uint8_t seq;				// Seq number of range message
		long long t_sec;    		// Epoch time in seconds
		long t_nsec;				// Nanoseconds into the Epoch		
	};
	
	//!Default constructor
	IdolinkDev(void)
	{
		m_portHandler = 0;
		m_binProtocol = false;
	}

	//!Default destructor
	~IdolinkDev(void)	
	{
		finish();
	}

	/**
	 Intialize the serial port to the given values.
	 The function open the serial port in read-write mode

	 \param devive Path to the serial device (e.g. '/dev/ttyUSB0')
	 
	 \return
	 - true: success
	 - false: error while open the device
	 */
	bool init(const std::string devive, bool binProtocol = false)
	{
		struct termios my_termios;

		// Make sure port is closed 
		if (m_portHandler > 0)
			close(m_portHandler);

		// Open the port in read-write mode 
		m_portHandler = open(devive.c_str(), O_RDWR | O_NOCTTY);
		if (m_portHandler < 0)
			return false;

		/* Get the port attributes and flush all data on queues*/
		tcgetattr(m_portHandler, &my_termios);
		tcflush(m_portHandler, TCIOFLUSH);

		/* Setup the communication */
		my_termios.c_iflag &= ~(BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | IXON
				| INLCR | IGNCR | ICRNL);
		my_termios.c_iflag |= IGNBRK | IXOFF;
		my_termios.c_oflag &= ~(OPOST);
		my_termios.c_cflag |= CLOCAL | CREAD;
		my_termios.c_cflag &= ~PARENB;
		my_termios.c_cflag |= CS8;
		my_termios.c_cflag &= ~CSTOPB;
		my_termios.c_cflag &= ~CRTSCTS;
		my_termios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | NOFLSH
				| TOSTOP | ISIG | IEXTEN);
		my_termios.c_cc[VMIN] = 0; //Each simple read call will be blocked until receive at least one byte

		//VTIME = Timeout. Is a character count ranging from 0 to 255 characters.
		//It is time measured in 0.1 second intervals, (0 to 25.5 seconds).
		//More information in http://www.unixwiz.net/techtips/termios-vmin-vtime.html

		my_termios.c_cc[VTIME] = 1;	//0 = No timeout for reading
		if(binProtocol)
		{
			m_binProtocol = true;
			cfsetispeed(&my_termios, B230400);
			cfsetospeed(&my_termios, B230400);
		}
		else
		{
			m_binProtocol = false;
			cfsetispeed(&my_termios, B115200);
			cfsetospeed(&my_termios, B115200);
		}
		tcsetattr(m_portHandler, TCSANOW, &my_termios);

		return true;
	}	

	//!Finish the serial communication (closes serial port)
	void finish(void)
	{
		if (m_portHandler > 0) 
		{
			close(m_portHandler);
			m_portHandler = 0;
		}
	}
	
	/**
	 * Read range message from tag 
	 *
	 * \return
	 * - -5: Reading error
	 * -  0: Success
	 **/
	int readRange(RangeData& data) 
	{
		if(m_binProtocol)
			return readRangeBin(data);
		else
			return readRangeChar(data);
	}
	
	
protected:

	/**
	 * Read range message from tag usin binary protocol
	 *
	 * \return
	 * - -5: Reading error
	 * -  0: Success
	 **/
	int readRangeBin(RangeData& data) 
	{
		int res, count;
		uint8_t buf[150], c;
		
		// Synch with the beginning of the messsage
		c = 0;
		while (c != 0xAA) 
		{
			if (read(m_portHandler, &c, 1) <= 0) 
				return -5;
		}
	
		// Read message block
		count = 1;
		buf[0] = 0xAA;
		while(count < 27)
		{
			res = read(m_portHandler, &c, 1);
			if(res <= 0) 
				return -5;
			if(res == 1) 
				buf[count++] = c;
		}
		
		// Parse message
		uint8_t stx = buf[0];
		uint8_t cmd = buf[1];
		uint8_t group = buf[2];
		uint8_t frame_type = buf[3];
		uint8_t euid[8];
		euid[0] = buf[11];
		euid[1] = buf[10];
		euid[2] = buf[9];
		euid[3] = buf[8];
		euid[4] = buf[7];
		euid[5] = buf[6];
		euid[6] = buf[5];
		euid[7] = buf[4];
		uint64_t euid_src = *((uint64_t *)euid);
		euid[0] = buf[19];
		euid[1] = buf[18];
		euid[2] = buf[17];
		euid[3] = buf[16];
		euid[4] = buf[15];
		euid[5] = buf[14];
		euid[6] = buf[13];
		euid[7] = buf[12];		
		uint64_t euid_relay = *((uint64_t *)euid);
		uint8_t src = buf[20];
		uint8_t dst = buf[21];
		uint8_t seq = buf[22];
		uint8_t req_res = buf[23];
		int8_t rssi = *((int8_t *)(buf+24));
		uint8_t hop = buf[25];
		uint8_t len = buf[26];
		
		// Check received values
		if(stx != 0xAA || cmd != 0x10)
			return -5;

		// Read sub-command block
		count = 0;
		while(count < len)
		{
			res = read(m_portHandler, &c, 1);
			if(res <= 0) 
				return -5;
			if(res == 1) 
				buf[count++] = c;
		}
		
		// Parse sub-command
		uint8_t sub = buf[0];
		uint8_t vn = buf[1];
		uint8_t bs = buf[2];
		uint8_t bl = buf[3];
		uint8_t loc_type = buf[4];
		uint8_t tag_seq = buf[5];
		uint8_t aux[8];
		aux[0] = buf[13];
		aux[1] = buf[12];
		aux[2] = buf[11];
		aux[3] = buf[10];
		aux[4] = buf[9];
		aux[5] = buf[8];
		aux[6] = buf[7];
		aux[7] = buf[6];
		double loc_data = *((double *)aux);
		aux[0] = buf[32];
		aux[1] = buf[31];
		int16_t rx_level = *((int16_t *)aux);
		int16_t blink = *((int16_t *)(buf+33));
		
		// Check received values
		if(sub != 0x8A || loc_type != 0x02 || loc_data < 0.0 || loc_data > 100.0)
			return -5;
		
		// Synch with the end of the messsage
		c = 0;
		while (c != 0x55) 
		{
			if (read(m_portHandler, &c, 1) <= 0) 
				return -5;
		}
		
		// Fill data structure				
		data.sourceId = euid_src;
		data.destinationId = euid_relay;
		data.rssi = rx_level*0.01;
		data.range = (float)loc_data;
		
		return 0; 
	}
	
	/**
	 * Read range message from tag using character protocol
	 *
	 * \return
	 * - -5: Reading error
	 * -  0: Success
	 **/
	int readRangeChar(RangeData& data) 
	{
		int res, count;
		char buf[256], c;
		struct timespec stamp;
		
		// Synch with the beginning of the messsage
		c = 0;
		while (c != 'a') 
		{
			if (read(m_portHandler, &c, 1) <= 0) 
				return -5;
		}
		buf[0] = c;
		clock_gettime(CLOCK_REALTIME, &stamp);
	
		// Read a complete message string
		c = 0;
		count = 1;
		while (c != '\n') 
		{
			res = read(m_portHandler, &c, 1);
			if(res <= 0) 
				return -5;
			if(res == 1) 
			{
				if(c == ':')	// Substitute ':' by spaces for easy string parsing
					c = ' ';
				buf[count++] = c;
			}
		}
		buf[count] = '\0';
		
		// Parse message
		char gar[25];
		sscanf(buf, "%s %lx %s %hhx %s %f %s %f", gar, &data.sourceId, gar, &data.seq, gar, &data.range, gar, &data.rssi);
		data.destinationId = 0;
		data.t_sec = stamp.tv_sec;
		data.t_nsec = stamp.tv_nsec;
		
		return 0; 
	}
	
	//!Serial port handler
	int m_portHandler;
	
	//!Binary or character protocol flag
	bool m_binProtocol;
};

#endif

