#include "mavlink_commands.hpp"
#include <thread>

std::shared_ptr<MAVLink> mavlink;

void rd(int n){
    while(1){
        mavlink->read_data();
    }
}

int main(int argc, char const *argv[])
{
    bool done = false;

    mavlink = std::make_shared<MAVLink>(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    mavlink->req_data_stream();

    std::thread t1(rd, 1);

    // mavlink->arm_disarm(true);

    mavlink->takeoff(5);

    // mavlink->set_mode(MAV_MODE_AUTO_ARMED);

    t1.join();

    return 0;
}
