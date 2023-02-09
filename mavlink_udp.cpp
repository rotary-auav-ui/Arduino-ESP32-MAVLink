#include "mavlink_commands.hpp"

int main(int argc, char const *argv[])
{
    std::shared_ptr<MAVLink> mavlink = std::make_shared<MAVLink>(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    mavlink->req_data_stream();

    // mavlink->arm_disarm(true);

    mavlink->takeoff(5);

    while(1){
        mavlink->read_data();
        // sleep(1);
    }

    return 0;
}
