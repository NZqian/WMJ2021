#include "./libFSM/include/StateMachine.h"

int main(int argc, char *argv[])
{
    StateMachine sm;
    std::thread sm_t(&StateMachine::stateMachineLoop, &sm);
    sm_t.join();
    return 0;
}
