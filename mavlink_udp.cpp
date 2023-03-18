#include "mavlink_commands.hpp"
#include <thread>
#include <array>

std::shared_ptr<MAVLink> mavlink;

void rd(int n){
    while(1){
        mavlink->send_heartbeat();
        mavlink->read_data();
    }
}

int main(int argc, char const *argv[]){
    bool done = false;

    mavlink = std::make_shared<MAVLink>(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    std::thread t1(rd, 1);

    mavlink->timeout(2);

    mavlink->set_fly_alt(3);

    mavlink->add_waypoint(47.3976479, 8.5459404);

    mavlink->add_waypoint(47.3978930, 8.5459663);

    mavlink->send_mission();

    t1.join();

    return 0;
}