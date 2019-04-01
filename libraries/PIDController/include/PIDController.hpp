/*
 * PIDController.hpp
 *
 *  Created on: 2019-03-26
 *      Author: liam
 */

#ifndef LIBRARIES_PIDCONTROLLER_INCLUDE_PIDCONTROLLER_HPP_
#define LIBRARIES_PIDCONTROLLER_INCLUDE_PIDCONTROLLER_HPP_

/****************************************
 * INCLUDES
 ****************************************/



/****************************************
 * CLASS DEFINITION
 ****************************************/

class PIDController
{

    public:

        PIDController();
        ~PIDController();

        /**
         * @param Kp, Proportional Gain
         * @param Ki, Integral Gain
         * @param Kd, Derivative Gain
         * @param max, maximum allowable value of the variable calculated
         * @param min, minimum allowable value of the variable calculated
         * @param dt, loop interval time
         */
        bool Initialize(double Kp, double Ki, double Kd, double max, double min, double dt);

        /**
         * @param SetPoint, is the desired value
         * @param ProcessValue, is the current value, Error is SetPoint - ProcessValue
         *
         * @returns the manipulated value
         */
         double Calculate(double SetPoint, double ProcessValue);

    private:

        double _kp;
        double _ki;
        double _kd;

        double _max;
        double _min;

        double _dt;

        double _integral;
        double _preError;
};




#endif /* LIBRARIES_PIDCONTROLLER_INCLUDE_PIDCONTROLLER_HPP_ */
