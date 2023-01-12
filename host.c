// C library headers
#include <stdio.h>
#include <string.h>
#include <pthread.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdlib.h>

#define file_serial_port "/dev/ttyACM0" //para comprobar añadir / al principio

char read_buffer[8];
pthread_t tp;

void* productor(char* f) {

  int serial_port, n_bytes;
  char temp_raw_str[5], ldr_raw_str[5];


  if ((serial_port = open(f, O_RDONLY)) < 0) {
	printf("Error %i from open: %s\n", errno, strerror(errno));
    //fprintf(stderr, "Error al abrir para lectura: %s\n", f);
    exit(3);
  }
  
  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      exit(3);
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 8;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      exit(4);
  }

  while(1)
  {
  //if((n_bytes = read(serial_port, &read_buffer, sizeof(read_buffer))) > 0) {
    n_bytes = read(serial_port, &read_buffer, sizeof(read_buffer));
    //read_buffer[8] = '\0';
    strncpy(temp_raw_str, read_buffer, 4);
    temp_raw_str[5] = '\0';
    printf("Valor Temp = %s\n", temp_raw_str);
    strncpy(ldr_raw_str, &read_buffer[4], 4);
    ldr_raw_str[5] = '\0';
    printf("Valor LDR = %s\n",ldr_raw_str);
    
    printf("Len read_buffer = %d\n", (int)strlen(read_buffer));
    printf("Size read_buffer = %d\n", sizeof(read_buffer));
	  printf("Read %i bytes. Received message: %s\n", n_bytes, read_buffer);
  };
  close(serial_port);
  pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
	memset(&read_buffer, '0', sizeof(read_buffer));
	if (pthread_create(&tp, NULL, (void *)productor, (void *)file_serial_port) != 0)
      {
          fprintf(stderr, "Error al crear thread para %s\n", file_serial_port);
          exit(2);
      }
	 pthread_join(tp, NULL);
	return 0;
}