#include <string>

#ifndef BAR_STATE_H
#define BAR_STATE_H

class BarState: public yasmin::State {
public:
    BarState();
    ~BarState();

    std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);

};

#endif // BAR_STATE_H