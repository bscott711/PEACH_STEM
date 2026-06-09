#pragma once
#include "tasks/StepperAxisNode.h"
#include "messaging.h"

class DishLiftNode : public StepperAxisNode {
protected:
    virtual bool checkInterlock(int desiredSpeed) override;

public:
    DishLiftNode();
    virtual ~DishLiftNode();
};
