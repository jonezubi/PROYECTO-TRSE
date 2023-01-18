// C library headers 

#include <stdio.h> 
#include <string.h> 
#include <pthread.h> 
#include <assert.h> 
#include <time.h> 

// Linux headers 

#include <fcntl.h> // Contains file controls like O_RDWR 
#include <errno.h> // Error integer and strerror() function 
#include <termios.h> // Contains POSIX terminal control definitions 
#include <unistd.h> // write(), read(), close() 
#include <stdlib.h>
 

#define file_serial_port "/dev/ttyACM0" //para comprobar añadir / al principio 

#define calor 0 
#define frio 1 
#define templado 2 
  
#define TEMP_RAW_COLD 850 // 18ºC 
#define TEMP_RAW_HOT 950 // 26ºC 
 
#define ON 1 
#define OFF 0 
#define limite_LDR 2000



pthread_t tp, tc;

char read_buffer[8] = ""; 
char buffer_aux[9] = ""; 

char temp_raw_str[5], ldr_raw_str[5]; 
int estado_temp, estado_temp_ant = -1; 
int estado_ldr_led, estado_ldr_led_ant = -1; 
int incendio=0; 
char *msg_estado_LDR[] = {"LUZ: OFF", "LUZ:  ON"}; 
char *msg_estado_temp[] = {"Ventilador:  ON | Calefactor: OFF", "Ventilador: OFF | Calefactor:  ON", "Ventilador: OFF | Calefactor: OFF"}; 
float temp;
int LDR; 
time_t t;
struct tm tm ;

void* productor(char* f) {
  FILE *log;
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

  while(strcmp(buffer_aux, "SAPAGADO") != 0)
  {
    //t = time(NULL);
    //tm = *localtime(&t);
    n_bytes = read(serial_port, &read_buffer, sizeof(read_buffer));
    strncpy(buffer_aux, read_buffer, 8);
    buffer_aux[9] = '\0';
    //printf("NO => %s\n", buffer_aux);

    if(strcmp(buffer_aux, "INCENDIO") == 0){
        incendio=1;
        //t = time(NULL);
        //tm = *localtime(&t);
        log = fopen ("log.txt", "a");
        fprintf(log, "%d-%02d-%02d %02d:%02d:%02d - Ha ocurrido un incendio\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        fclose(log);

        while(strcmp(buffer_aux, "IAPAGADO") != 0)
        {
            read(serial_port, &read_buffer, sizeof(read_buffer));
      	    strncpy(buffer_aux, read_buffer, 8);
            buffer_aux[9] = '\0';
            //printf("Incendio => %s\n", buffer_aux);
        }
        //t = time(NULL);
        //tm = *localtime(&t);
	    log = fopen ("log.txt", "a");
        fprintf(log,"%d-%02d-%02d %02d:%02d:%02d - Incendio apagado\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec); 
        fclose(log);
        incendio=0;
    }
    else if(strcmp(buffer_aux, "IAPAGADO") != 0)
    {
        strncpy(temp_raw_str, buffer_aux, 4);
        temp_raw_str[5] = '\0';
        int temp_ADC = atoi(temp_raw_str);
        float temp_volt = ((temp_ADC * 3.3)/4095 ) - 0.5;
        temp = temp_volt/0.01;
        
        strncpy(ldr_raw_str, &buffer_aux[4], 4);
        ldr_raw_str[5] = '\0';
        

        LDR=atoi(ldr_raw_str);

        // msg_estado_temp Temperatura
        if(atoi(temp_raw_str) < TEMP_RAW_COLD) // Frio
        {
            estado_temp = frio;
        }
        else if (atoi(temp_raw_str) > TEMP_RAW_HOT) // Calor
        {
            estado_temp = calor;
        }
        else // Templado
        {
            estado_temp = templado;
        }

        // Claro
        if(LDR  < limite_LDR){
            estado_ldr_led = OFF;
        }
        // Oscuro
        else {
            estado_ldr_led = ON;
        }

        // Check cambio de estado
        if(estado_temp != estado_temp_ant){
            log = fopen ("log.txt", "a");
            //printf("%d-%02d-%02d %02d:%02d:%02d - %s\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, msg_estado_temp[estado_temp]);
            fprintf(log,"%d-%02d-%02d %02d:%02d:%02d - %s\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, msg_estado_temp[estado_temp]);
            fclose(log);
        }
        estado_temp_ant = estado_temp;

        // Check LDR switch
        if (estado_ldr_led != estado_ldr_led_ant){
            log = fopen ("log.txt", "a");
            fprintf(log,"%d-%02d-%02d %02d:%02d:%02d - %s\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, msg_estado_LDR[estado_ldr_led]);
            fclose(log);
	    }
	    estado_ldr_led_ant = estado_ldr_led;
    }
  };
  close(serial_port);
  pthread_exit(NULL);
}

 
 

void* consumidor() 
{ 
    int widthLDR=5;
    while(strcmp(buffer_aux, "SAPAGADO") != 0) 
    { 

        t = time(NULL);
        tm = *localtime(&t);
        char s[64];
        size_t ret = strftime(s, sizeof(s), "%c", &tm);
        assert(ret);
        /*print menu screen*/ 
        system("clear");
        if (incendio==1)
        {
            printf("\t\t\t                             s \n");
            printf("\t\t\t                            s \n");
            printf("\t\t\t                 /#\\     s  s\n"); 
            printf("\t\t\t               /#####\\   ___\n");
            printf("\t\t\t             /#########\\|   |\n");
            printf("\t\t\t           /#############\\  |\n");
            printf("\t\t\t         /#################\\|\n");
            printf("\t\t\t       /### DOMOTIC  HOME ###\\\n");
            printf("\t\t\t     /#########################\\\n");
            printf("\t\t\t   /#############################\\\n");
            printf("\t\t\t /#################################\\\n");
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n");    
            printf("\t\t\t|!!!! %s !!!!!|\n", s);
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n"); 
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n");
            printf("\t\t\t|!!!!!!!!!!   INCENDIO!   !!!!!!!!!!|\n"); 
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n");   
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n");   
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n");    
            printf("\t\t\t|!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!|\n");    
        }


        else
        {
            printf("\t\t\t                              \n");
            printf("\t\t\t                            \n");
            printf("\t\t\t                 /#\\\n");
            printf("\t\t\t               /#####\\   ___\n");
            printf("\t\t\t             /#########\\|   |\n");
            printf("\t\t\t           /#############\\  |\n");
            printf("\t\t\t         /#################\\|\n");
            printf("\t\t\t       /### DOMOTIC  HOME ###\\\n");
            printf("\t\t\t     /#########################\\\n");
            printf("\t\t\t   /#############################\\\n");
            printf("\t\t\t /#################################\\\n");
            printf("\t\t\t|***********************************|\n"); 
            printf("\t\t\t|**** %s *****|\n", s);
            printf("\t\t\t|***********************************|\n"); 
            printf("\t\t\t|\tTemperatura: %6.2f ºC      |\n",temp);
            printf("\t\t\t| %s |\n",msg_estado_temp[estado_temp]);    
            printf("\t\t\t|***********************************|\n");   
            printf("\t\t\t|\tLuminosidad: %*d lux      |\n", widthLDR,LDR);
            //printf("\t\t\t|\tLuminosidad: %6.2f lux     |\n", LDR);
            printf("\t\t\t|               %s            |\n",msg_estado_LDR[estado_ldr_led]);  
            printf("\t\t\t|___________________________________|\n");
        }

        sleep(1); 
    } 

    pthread_exit(NULL); 
} 
  



int main(int argc, char *argv[]) 
{ 
    FILE *log;
    t = time(NULL);
    tm = *localtime(&t); 
    log = fopen ("log.txt", "w+");
    fprintf(log,"%d-%02d-%02d %02d:%02d:%02d - Sistema encendido\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(log);
    //memset(&read_buffer, '0', sizeof(read_buffer)); 
    //memset(&buffer_aux, '\0', sizeof(buffer_aux)); 
    
    if (pthread_create(&tp, NULL, (void *)productor, (void *)file_serial_port) != 0) 
    { 
        fprintf(stderr, "Error al crear thread para %s\n", file_serial_port); 
        exit(2);
    } 

    if (pthread_create(&tc, NULL, (void *)consumidor, NULL) != 0) 
    { 
        fprintf(stderr, "Error al crear thread consumidor \n"); 
        exit(2); 
    } 

    pthread_join(tp, NULL); 
    pthread_join(tc, NULL);
    t = time(NULL);
    tm = *localtime(&t); 
    log = fopen ("log.txt", "a");
    fprintf(log,"%d-%02d-%02d %02d:%02d:%02d - Sistema apagado\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    fclose(log);

    return 0; 
} 