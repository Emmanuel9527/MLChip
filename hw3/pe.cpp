#include "pe.h"
#include <sstream>
#include <fstream>
#include <string>

static int all_pass_count = 0;
#define get_cycle() int(sc_time_stamp().to_seconds() * 1e9 / 10)
void print_fail();
void print_pass();

PE::PE(){}

void PE::init(int pe_id)
{
    id = pe_id;
    send_count = 0;
    recv_count = 0;
    
    std::ostringstream oss;
    oss << id;
    std::ifstream file(("pattern/core_" + oss.str() + ".txt").c_str());
    string type;
    int another_id, vec_len;
    float data;
    while(file >> type)
    {
        Packet packet;
        file >> another_id;
        file >> vec_len;
        for(int i=0; i<vec_len; i++)
        {
            file >> data;
            packet.datas.push_back(data);
        }
        if(type == "TO")
        {
            packet.source_id = id;
            packet.dest_id = another_id;
            send_packets.push_back(packet);
        }
        else // type == "FROM"
        {
            packet.source_id = another_id;
            packet.dest_id = id;
            recv_packets.push_back(packet);
        }
    }
    file.close();
    send_count = 0;
    recv_count = 0;
}

Packet* PE::get_packet()
{
    if(send_count == send_packets.size())
        return NULL;
    if(DEBUG_MODE)
        printf("PE_%02d: Send  %2d th packet at %d th cycle.\n", id, send_count+1, get_cycle());
    Packet* p = new Packet(send_packets[send_count]);
    send_count++;
    return p;
}

void PE::check_packet(Packet* p)
{
    int vec_len;
    if(recv_count == TOTAL_PACKET_NUM)
    {
        print_fail();
        printf("PE_%02d: Received too many packets.\n", id);
        exit(1);
    }
    if(DEBUG_MODE)
        printf("PE_%02d: Check %2d th packet at %d th cycle.\n", id, recv_count+1, get_cycle());
    // ------------------
    // check packet info
    // ------------------
    bool pass = true;
    int idx;
    for(idx=0; idx<TOTAL_PACKET_NUM; idx++)
        if(recv_packets[idx].source_id == p->source_id)
            break;
    
    if( p->source_id     != recv_packets[idx].source_id ||
        p->dest_id       != recv_packets[idx].dest_id ||
        p->datas.size()  != recv_packets[idx].datas.size() )
        pass = false;

    if(pass)
    {
        vec_len = p->datas.size();
        for(int i=0; i<vec_len; i++) 
        {
            if(p->datas[i] != recv_packets[idx].datas[i]) {
                pass = false;
                break;
            }
        }
    }

    if(!pass)
    {
        print_fail();
        printf("PE_%02d: %d th received packet is wrong!\n", id, recv_count+1);
        printf("Your packet: (%d --> %d)\n", p->source_id, p->dest_id);
        vec_len = p->datas.size();
        for(int i=0; i<vec_len; i++)
        {
            if(i >= recv_packets[idx].datas.size() || p->datas[i] != recv_packets[idx].datas[i])
                printf("\033[0m\033[1;31m%4.3f\033[0m  ", p->datas[i]);
            else
                printf("%4.3f  ", p->datas[i]);
            if(i % 10 == 9)
                cout << endl;
        }
        printf("\n---\n");
        printf("Golden packet: (%d --> %d)\n", recv_packets[idx].source_id, recv_packets[idx].dest_id);
        vec_len = recv_packets[idx].datas.size();
        for(int i=0; i<vec_len; i++)
        {
            printf("%4.3f  ", recv_packets[idx].datas[i]);
            if(i % 10 == 9)
                cout << endl;
        }
        printf("\n");
        sc_stop();
        exit(1);
    }
    recv_count++;
    if(recv_count == TOTAL_PACKET_NUM)
    {
        all_pass_count ++;
        if(all_pass_count == 16)
        {
            print_pass();
            sc_stop();
        }
    }
}

void print_fail()
{
    string oops_str = "\033[0m\033[1;31mOOPS !!\033[0m           ";
    
    ostringstream cycle_stream;
    cycle_stream << get_cycle();  
    string cycle_str = "at " + cycle_stream.str() + " th cycle"; 

    cout << endl;
    cout << "  ============================                  " << endl;
    cout << "  +                          +         |\\__|\\  " << endl;
    cout << "  +   " << oops_str << "     +        / X,X  |  " << endl;
    cout << "  +                          +      /_____   |  " << endl;
    cout << "  +   Simulation failed      +     /^ ^ ^ \\  | " << endl;
    cout << "  +   " << left << setw(20) << cycle_str << "   +    |^ ^ ^ ^ |w| " << endl;
    cout << "  +                          +     \\m___m__|_|" << endl;
    cout << "  ============================                 " << endl;
    cout << endl;
}

void print_pass()
{
    string pass_str = "\033[0m\033[1;32mCongratulations !!\033[0m";
    
    
    ostringstream cycle_stream;
    cycle_stream << get_cycle();  
    string cycle_str = "at " + cycle_stream.str() + " th cycle"; 

    cout << endl;
    cout << "  ============================                  " << endl;
    cout << "  +                          +         |\\__|\\ " << endl;
    cout << "  +   " << pass_str << "     +        / O.O  |  " << endl;
    cout << "  +                          +      /_____   |  " << endl;
    cout << "  +   Simulation completed   +     /^ ^ ^ \\  | " << endl;
    cout << "  +   " << left << setw(20) << cycle_str << "   +    |^ ^ ^ ^ |w| " << endl;
    cout << "  +                          +     \\m___m__|_| " << endl;
    cout << "  ============================                  " << endl;
}