/*RPI <-> Arduino Communication Protocol Data Packet definitions(Serial Communication)

Data Packet: <HEADER_TYPE><COMMAND><DATA_TYPE><ELEMENT><NUMBER_DATA_BYTES><DATA><TERMINATOR>

Size:
HEADER_TYPE: 1Byte					Type of communication
COMMAND: 1Byte						Instructions to the Arduino
DATA_TYPE: 1Byte					String or a number data type
NUMBER_DATA_BYTES: 1Byte			Only used with STRING
ELEMENT: 1Byte						Element to be interacted with
DATA: Variable						Data
TERMINATOR: 1Byte					Terminator Byte


*/

//HEADER_TYPE
#define CONFIG	0x00
#define STATE	0x01



//COMMAND
#define SET		0x00
#define GET		0x01


//DATA_TYPE
#define NUMBER 0x00
#define STRING 0x01

//ELEMENT
#define AMBIENT_LIGHT 0x00


//TERMINATOR
#define TERMINATOR 0xFF


