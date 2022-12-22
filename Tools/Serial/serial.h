// Taken and adjusted from https://github.com/EMCLab-Sinica/Intermittent-OS
#define DEBUG 1

#define SERIAL_CLK_1MHZ 0
#define SERIAL_CLK_8MHZ 6

// Init serial
void uartinit(char clock_setting);
// Serial printf
void print2uart(char* format,...);
// Serial print single character
void print2uartc(char c);
// dummy function
void dummyprint(char* format,...);
// Serial printf for debugging

//void dprint2uart(char* format,...);
//Put a string to serial
void print2uartlength(char* str,int length);
//Convert integer to a string
char *convert(unsigned int num, int base);
//Convert long integer to a string
char *convertl(unsigned long num, int base);
