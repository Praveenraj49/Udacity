#include <numeric>
#include <limits>
#include <iostream>
#include <vector>

#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    
}

void PID::UpdateError(double cte) {
    d_error = cte-p_error;
    p_error = cte;
    i_error += cte;
    
    /*cout << "Starting to twiddle" << endl;
    double tolerance = 0.02;
    double best_error = std::numeric_limits<double>::max();
    Twiddle(tolerance , best_error);
    cout << "Best Error after Twiddling " << best_error << endl; */
}

double PID::TotalError() {
    double total_err=0.0;
    total_err = -Kp * p_error - Ki * i_error - Kd * d_error;
    
    return total_err;
}

void PID::Twiddle(double tolerance , double& best_error)
{
    vector<double> p(3);
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
   
    vector<double> dparams(3);
    dparams[0] = 1.0;
    dparams[1] = 1.0;
    dparams[2] = 1.0;
    
    double sum = std::accumulate(dparams.begin(), dparams.end() ,0);
    double total_error = TotalError();
    int it =0;
    cout << "Initial Total error : " << total_error << endl;
    while(sum > tolerance)
    {
        for(int i =0;i<p.size();++i)
        {
            p[i] +=dparams[i];
            switch(i){
                case 0:
                    Kp = p[i];
                    break;
                case 1:
                    Ki = p[i];
                    break;
                case 2:
                    Kd = p[i];
                    break;
                default:
                    break;
            }
            total_error = TotalError();
            if(total_error < best_error)
            {
                best_error = total_error;
                dparams[i] *= 1.1;
            }
            else
            {
                p[i] -= 2*dparams[i];
                switch(i){
                    case 0:
                        Kp = p[i];
                        break;
                    case 1:
                        Ki = p[i];
                        break;
                    case 2:
                        Kd = p[i];
                        break;
                    default:
                        break;
                }
                total_error = TotalError();
                if(total_error < best_error)
                {
                    best_error = total_error;
                    dparams[i] *= 1.1;
                }
                else
                {
                    p[i] += dparams[i];
                    dparams[i] *=0.9;
                }
                
            }
         cout <<"Iteration : " << it << " Proportional :" << Kp << " Integral :" << Ki << " Differential :"<< Kd << endl;
         cout << "Best Error: " << best_error << endl;

        }
        sum = std::accumulate(dparams.begin(), dparams.end() ,0);
        it++;

        
    }
   
}

