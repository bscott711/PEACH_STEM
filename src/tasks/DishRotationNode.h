#pragma once
#include "tasks/StepperAxisNode.h"
#include "messaging.h"

class DishRotationNode : public StepperAxisNode {
protected:
    virtual bool checkInterlock(int desiredSpeed) override;

public:
    DishRotationNode();
    virtual ~DishRotationNode();
};
