#ifndef PE_H
#define PE_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <algorithm>
#include <vector>
#include "systemc.h"
using namespace std;

#define DEBUG_MODE false
#define TOTAL_PACKET_NUM 1

struct Packet {
    int source_id;
    int dest_id;
    vector<float> datas;
};

class PE {
public:
    PE();
    Packet* get_packet();
    void check_packet(Packet* p);
    void init(int pe_id);
    
private:
    int id;
    int send_count;
    int recv_count;
    vector<Packet> send_packets;
    vector<Packet> recv_packets;
};

#endif