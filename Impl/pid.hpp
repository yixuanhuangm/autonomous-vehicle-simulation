#ifndef PID_HPP
#define PID_HPP

class PIDController {
public:
    void setPID(double kp, double ki, double kd, double dt);

    void Setpoint(double setpoint);

    void setOutputLimits(double minOutput, double maxOutput);

    double compute(double input);

    void Print();

private:
    double kp;
    double ki;
    double kd;
    double dt;
    double setpoint;
    double minOutput;
    double maxOutput;
    double integralTerm;
    double lastError;
};

inline void PIDController::setPID(double kp, double ki, double kd, double dt) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->setpoint = 0.0;
    this->minOutput = 0.0;
    this->maxOutput = 1.0;
    this->integralTerm = 0.0;
    this->lastError = 0.0;
}


void PIDController::Setpoint(double setpoint) {
    this->setpoint = setpoint;
}

void PIDController::setOutputLimits(double minOutput, double maxOutput) {
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
}

double PIDController::compute(double input) {
    double error = setpoint - input;
    double proportionalTerm = kp * error;
    integralTerm += ki * error * dt;
    double derivativeTerm = kd * (error - lastError) / dt;
    lastError = error;

    double output = proportionalTerm + integralTerm + derivativeTerm;
    if (output < minOutput) {
        output = minOutput;
    } else if (output > maxOutput) {
        output = maxOutput;
    }
    return output;
}

void PIDController::Print() {
    std::cout<<"targetspeed=="<<this->setpoint<<endl;
}



#endif 