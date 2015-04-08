//============================================================================
#include "SocketComm.h"
using namespace std;

runtime_error SocketComm::CreateSocketError()
{
	std::ostringstream temp;
	int errno = 0;
	temp << "Socket Error #" << errno << ": " << strerror(errno);
	return std::runtime_error(temp.str());
}

void SocketComm::sendAll(int socket, const char* const buf, const int size)
{
	int bytesSent = 0;
	do
	{
		int result = send(socket, buf + bytesSent, size - bytesSent, 0);
		if(result < 0)
		{
			throw CreateSocketError();
		}
		bytesSent += result;
	} while(bytesSent < size);
}

void SocketComm::GetLine(int socket, std::stringstream& line)
{
	for(char c; recv(socket, &c, 1, 0) > 0; line << c)
	{
		if(c == '\n')
		{
			return;
		}
	}
	throw CreateSocketError();
}
string SocketComm::send_and_receive(int socket,  const char* command, int commandlength)
{
	sendAll(socket, command, commandlength);
	stringstream line;
	line.clear();
	GetLine(socket, line);
	return line.str();
}


SocketComm::SocketComm() {
	cout << "Initializing: ";
	sckt1 = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sckt1 <= 0)
		cout << "Error, no socket available" << endl;
	cout << ".";
	sockaddr_in service; // Normale IPv4 Struktur
	service.sin_family = AF_INET; // AF_INET für IPv4, für IPv6 wäre es AF_INET6
	service.sin_port = htons(4000);
	service.sin_addr.s_addr = inet_addr("127.0.0.1");
	cout << ".";
	int result = connect(sckt1, reinterpret_cast<sockaddr*>(&service), sizeof(service));
	if (result == -1)
		cout << "Error, connection refused" << endl;
	last_collision_timestamp.resize(3);
	last_collision_timestamp[0] = 0;
	last_collision_timestamp[1] = 0;
	last_collision_timestamp[2] = 0;
	cout << "done! \n";
	/*char command[30] = "id1 simulation list_streams \n";

	string line = send_and_receive(sckt1, command, strlen(command));
	cout << line << endl;

	char command2[37] = "id1 robot.armature list_IK_targets \n";
	line = send_and_receive(sckt1, command2, strlen(command2));
	cout << line << endl;*/
	// x: 1,3 - 2.0  y:-0.4-0.4, z: 1.9-2.4
	//char command3[114] =   "id1 robot.armature place_IK_target [\"name\":\"ik_target.robot.armature.kuka_7\", \"translation\":[1,1,2.2]] \n";
	//char command3[113]   =   "id1 robot.armature place_IK_target [\"ik_target.robot.armature.kuka_7\", [1.8, 0, 1.5], [0,0,0], false] \n";

}
void SocketComm::sendPosition(float X, float Y, float Z)
{
	std::ostringstream stringStream;
	stringStream << "id1 robot.armature place_IK_target [\"ik_target.robot.armature.kuka_7\", [";
	stringStream << X;
	stringStream << ", ";
	stringStream << Y;
	stringStream << ", ";
	stringStream << Z;
	stringStream << "], [-1.6,0.4,0], false] \n";
	
	char* command4 = new char[stringStream.str().size()+1];
	//string command4str = stringStream.str();
	strcpy(command4, stringStream.str().c_str());

	/*string line =*/ send_and_receive(sckt1, command4, strlen(command4));

}
bool SocketComm::requestCollisionState()
{
	for (int i =0; i < 3; i++)
	{	
		std::ostringstream stringStream;
		stringStream << "id1 robot"<< i+2 <<".collision"<< i+1 <<" get_local_data\n";
		char* command5 = new char[stringStream.str().size()+1];
		strcpy(command5, stringStream.str().c_str());
	    string return_value = send_and_receive(sckt1, command5, strlen(command5));
	    std::cout  << std::endl << "On robot nr. " << i <<" received state:"; 
		std::cout << std::endl << return_value <<  std::endl;
		std::size_t string_position_timestamp = return_value.find("timestamp");
		std::size_t string_position_collision = return_value.find("collision");
		std::string timestamp = return_value.substr(string_position_timestamp+12, string_position_collision-string_position_timestamp-15);
		
		
		stringstream Str;
		Str << fixed << std::setprecision( 16 ) << timestamp;
		
		double current_timestamp;
		Str >> current_timestamp;
		cout.precision(15);
		//std::cout << "timestamp value: " << current_timestamp<< " from "<< timestamp <<"\n";
		if (current_timestamp > last_collision_timestamp[i])
		{
			last_collision_timestamp[i] = current_timestamp;
			return true;
		}
	}
	return false;
}

void SocketComm::sendGrasp(bool grasp)
{
	std::ostringstream stringStream;
	if (grasp == true)
		stringStream << "id1 robot.armature.gripper grab\n";
	else 
		stringStream << "id1 robot.armature.gripper release\n";
	
	char* command4 = new char[stringStream.str().size()+1];
	//string command4str = stringStream.str();
	strcpy(command4, stringStream.str().c_str());

	/*string line =*/ std::cout << send_and_receive(sckt1, command4, strlen(command4));

}
