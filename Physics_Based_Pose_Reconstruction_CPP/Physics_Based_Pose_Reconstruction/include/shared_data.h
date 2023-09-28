#pragma once
#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdlib>
#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")


extern mjData* d;				//defined and updated in main.cpp
extern mjModel* m;				//defined in main.cpp
extern mjvCamera cam;			//abstract camera
extern mjvOption opt;			//visualization options
extern mjvScene scn;			//abstract scene
extern mjrContext con;			//custom GPU context

extern SOCKET client_socket;    //client socket (python)
extern SOCKET server_socket;    //server socket

//data needed to be shared across different programs.....Make sure to define the variables in some file.
static void updateSharedData(mjData* d, mjModel* m, mjvCamera cam, mjvOption opt, mjvScene scn, mjrContext con) {
    d = d;
    m = m;
    cam = cam;
    opt = opt;
    scn = scn;
    con = con;
}

//intitalizes the server and client sockets
static bool init_sockets() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
        return 0;
    }

    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == INVALID_SOCKET) {
        std::cerr << "Socket creation failed\n";
        WSACleanup();
        return 0;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(8888);

    if (bind(server_socket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Socket binding failed\n";
        closesocket(server_socket);
        WSACleanup();
        return 0;
    }

    if (listen(server_socket, 1) == SOCKET_ERROR) {
        std::cerr << "Listen failed\n";
        closesocket(server_socket);
        WSACleanup();
        return 0;
    }

    std::cout << "Server waiting for a connection...\n";

    client_socket = accept(server_socket, nullptr, nullptr);
    if (client_socket == INVALID_SOCKET) {
        std::cerr << "Accept failed\n";
        closesocket(server_socket);
        WSACleanup();
        return 0;
    }

    std::cout << "Client connected!\n";
    return 1;
}