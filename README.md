Installation and use of T7logger

Installation of Arduino IDE and the extra libraries

1.	Download Arduino and install the stable version of Arduino IDE for your operation system:
https://www.arduino.cc/en/Main/Software
Accept all standard setting and install all of the required add-ons
2.	Start Arduino 
3.	Open the attached sketch (.ino)
4.	In File>Preferences find the empty field called “Additional Boards Manager URL” and paste this link there: http://arduino.esp8266.com/stable/package_esp8266com_index.json
5.	Open board manager from Tools>Board:Arduino/Genunino Uno>Boards Manager
6.	Search for esp8266, it will find only one result, click install
7.	Go to https://github.com/Seeed-Studio/CAN_BUS_Shield, click download or clone and download the zip file. Then extract it here: C:\Users\YourUserNmae\Documents\Arduino\libraries
8.	Go to https://github.com/PaulStoffregen/Time, click download or clone and download the zip file. Then extract it here: C:\Users\YourUserNmae\Documents\Arduino\libraries
9.	Replace the mcp_can.h and mcp_can_dfs.h files in C:\Program Files (x86)\Arduino\libraries\CAN_BUS_Shield-master with the ones provided in this project
10.	Go to Tool>Board and select NodeMCU 1.0
11.	Go to Tool>Board and select the appropriate com port
12.	Go to Tool>Board and select the highest upload speed
13.	Restart Arduino IDE
14.	Open the Sketch and click the upload button (right arrow). If everything went well it will start to flash the logger.









Usage
Setting up the variables to be logged
The logger always has the values of RPM, throttle position sensor (TPS), Speed (SPD), Engine temperature (Teng) and atmospheric pressure (Patm). These values are “flying” on the CANbus and can therefore just be sniffed and there is no need to actively log them, which is much creates traffic on the can network and increases the overhead of the ECU.

Changing the parameters that need to be actively logged happen only inside this block designated by those two lines:
// Change parameters here
// End of parameter changing block

Below a line-by-line explanation follows:
const unsigned int SizeLog = 12;   The total number of actively logged parameters
unsigned long Addr[SizeLog]  =             {15746224, 15746938, 15752804, 15752290, 15752446, 15752422, 15752874, 15749636, 15750536, 15750440, 15752952, 15750906};  These are the addresses of the values that we want to log. They are different for each software version (bin). To get the addresses open T7Suite. Go to File>Settings and uncheck “Display address and lengths in hex”. Then search in the symbol list and search for the symbol name (e.g. in.n_Engine for RPM, MAF.m_airInlet for airmass, etc.). Copy the address to this line, separated by a comma as in the example
const unsigned int Length[SizeLog] =       {2,        2,        2,        2,        2,        2,        4,        2,        2,        2,        1,        2}; This line is the length of the address, take this from the symbol list, exactly like you did with the address
const String         Name[SizeLog]  =      {"mReq",  "mAir",    "BCV",    "Pinl",   "O2Sf",   "Tair",   "Tinj",   "Amul",   "Lamb",   "LSwi",   "ECMS",   "Ioff"};  Write some name for the logged variable, following the example template
float              PollFreq[SizeLog] =     {10,       0.001,      10  ,    10   ,    0.001,      2,        10,     20,       10,       10,      10  ,     10};  This line represents the interval of polling of the variables. For example, the first variable is polled every 10 seconds, the second one every millisecond, etc. Of course the logger cannot log it every millisecond so it will do its best and poll it very fast. The 10 second interval ca be used for example in temperatures, which do not change much, or to quickly disable the logging of a variable, if it is not needed. 



Download and install Putty to save the logs while driving
https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html
Create a profile:
1.	Connection type: Serial
2.	Speed: 230400
3.	Serial Line: COM3 (set to the correct number)
4.	Session>Logging Log file name: PuttyLog&D&T.csv
5.	Session, Saved sessions: ArduinoLoggerCom3
6.	Click Save
After this has been setup, you just need to double click on “ArduinoLoggerCom3” and the logging will start


