// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/agents/agents.h"

//#include "pnc/agents/sample/sample_agent.h"

#include "pnc/agents/hongfz16/hongfz16_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
static simulation::Registrar<::hongfz16::SimpleVehicleAgent> registrar("hongfz16_agent");
