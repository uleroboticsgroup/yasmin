#include <string>

#ifndef YASMIN_DEMOS_FOO_STATE_H
#define YASMIN_DEMOS_FOO_STATE_H

class FooState: public yasmin::State {
public:
    FooState();
    ~FooState();

    std::string execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);
    int counter;

};

#endif // YASMIN_DEMOS_FOO_STATE_H