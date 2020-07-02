#include "HuberInstance.h"


namespace assistance_policy {

double HuberInstance::HUBER_HUMAN_ACTION_COST_MULTIPLIER = 0.2;
double HuberInstance::HUBER_DELTA_SWITCH_MODES = 0.075;  //point at which it goes from quadratic to linear
double HuberInstance::HUBER_CONSTANT_COST_ADD = 7.0;

double HuberInstance::HUBER_LINEAR_COST_MULTIPLIER_TOTAL = HUBER_HUMAN_ACTION_COST_MULTIPLIER + HUBER_CONSTANT_COST_ADD;
double HuberInstance::HUBER_QUADRATIC_COST_MULTPLIER = HUBER_HUMAN_ACTION_COST_MULTIPLIER/HUBER_DELTA_SWITCH_MODES; //multiplier comes out to this for quadratic part, calculated to make derivative smooth at mode switch
double HuberInstance::HUBER_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * HUBER_QUADRATIC_COST_MULTPLIER;
double HuberInstance::HUBER_LINEAR_COST_SUBTRACT = HUBER_HUMAN_ACTION_COST_MULTIPLIER * HUBER_DELTA_SWITCH_MODES * 0.5; //this is what we subtract off from linear part, calculated to make it connect to quadratic part



}
