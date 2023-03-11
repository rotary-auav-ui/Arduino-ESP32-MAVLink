#include "mavlink_commands.hpp"
#include <thread>

std::shared_ptr<MAVLink> mavlink;

void rd(int n){
    while(1){
        mavlink->read_data();
    }
}

int main(int argc, char const *argv[]){
    bool done = false;

    mavlink = std::make_shared<MAVLink>(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    mavlink->req_data_stream();

    std::thread t1(rd, 1);

    mavlink->timeout(3);

    mavlink->add_waypoint(47.3976479, 8.5459404, 5);

    mavlink->add_waypoint(47.3978930, 8.5459663, 5);

    mavlink->send_mission_count();

    mavlink->timeout(2);

    mavlink->start_mission();

    t1.join();

    return 0;
}