#include <iostream>
#include <cstring>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <regex>


using namespace std;

int ki,kp,kd;

void recvloop(int clientSocket){
    char buffer[1024] = {0};
    regex intTupleRegex(R"(\(\s*-?\d+\s*,\s*-?\d+\s*,\s*-?\d+\s*\))");
    while (true) {
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (regex_match(buffer, intTupleRegex)) { 
                int a, b, c;
            buffer[bytesReceived] = '\0'; // null-terminate the string

            if (sscanf(buffer, "(%d,%d,%d)", &a, &b, &c) == 3) {
                kp = a;
                ki = b;
                kd = c;
                cout << "Parsed values: kp=" << kp << ", ki=" << ki << ", kd=" << kd << endl;
            }
        }
        else{
            cout << "Wrong format" <<std::endl;
        }
    }   
}

int main() {
    // Step 1: Create socket
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket < 0) {
        cerr << "Error creating socket\n";
        return 1;
    }

    // Step 2: Define server address
    sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(12000); // Port number

    // Convert IP from text to binary
    if (inet_pton(AF_INET, "192.168.70.169", &serverAddress.sin_addr) <= 0) {
        cerr << "Invalid address or address not supported\n";
        return 1;
    }

    // Step 3: Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        cerr << "Connection failed\n";
        return 1;
    }

    cout << "Connected to server at 192.168.1.245:12000\n";
    
    thread reciever(recvloop,clientSocket);

    // Step 4: Send a message
    while(true){
        sleep(1);
        cout << "Parsed values: kp=" << kp << ", ki=" << ki << ", kd=" << kd << endl;
    }

    // Step 6: Close the socket
    close(clientSocket);
    return 0;
}
