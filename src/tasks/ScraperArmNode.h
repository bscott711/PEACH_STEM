#pragma once
#include "tasks/StepperAxisNode.h"
#include "messaging.h"

class ScraperArmNode : public StepperAxisNode {
protected:
    virtual bool checkInterlock(int desiredSpeed) override;

public:
    ScraperArmNode();
    virtual ~ScraperArmNode();
};
