#include <sys/socket.h> // socket(), connect()
#include <arpa/inet.h> // sockaddr_in
#include <iostream>
#include <string.h>
#include <stdexcept>
#include <sstream>
#include <math.h>
#include <cstring>
#include <iomanip>
#include <vector>
class SocketComm {
	public:
	SocketComm();
	std::runtime_error CreateSocketError();
	void sendAll(int socket, const char* const buf, const int size);
	void GetLine(int socket, std::stringstream& line);
	std::string send_and_receive(int socket,  const char* command, int commandlength);
	void sendPosition(float X, float Y, float Z);

	bool requestCollisionState();
	void sendGrasp(bool grasp);
	
	int sckt1;
	std::vector<double> last_collision_timestamp;
};
