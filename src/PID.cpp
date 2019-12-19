#include <limits>
#include <iostream>
#include <vector>
#include <math.h>
#include "PID.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
std::default_random_engine gen;

/**
 * TODO: Comlete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(string id_str, double Kp_, double Ki_, double Kd_, double hi_bnd, double lo_bnd, int tw_run_iter, double max_ch_rate, double act_res)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  dp = {0.099, 0.0099, 0.00099};
  tolerance = 0.0001;
  pIndex = 0;
  tw_state = 0;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
  hi_bound = hi_bnd;
  lo_bound = lo_bnd;
  params = {Kp, Ki, Kd};
  errors = {p_error, i_error, d_error};
  best_err = 100000000000.0;
  tw_err = 0.0;
  prev_speed = 0.0;
  prev_angle = 0.0;
  max_change_rate = max_ch_rate;
  act_resolution = act_res;
  scheduled_act.clear();
  prev_action = 0.0;
  prev_angle = 0.0;
  id = id_str; 
  run_prev_result = 0.0;
  run_iteration = tw_run_iter;
  run_p = params;
  run_state = 0;
  run_error = 0.0;
  run_n = 0;
  run_2n = 0;
  run_i = 0;
  run_prev_cte = 0.0;
  run_int_cte = 0.0;
  run_running = false;
  run_stop  = false;
  enable_twiddle = false;
}

void PID::initRun(const vector<double>& p, int n)
{
  if (!run_running) {
    run_p = p;
    run_state = 0;
    run_error = 0.0;
    run_n = n;
    run_2n = n*2;
    run_i = 0;
    run_prev_cte = p_error;
    run_int_cte = 0.0;
    run_running = true;
    run_stop = false;
  }
}

vector<double> PID::getParams()
{
  return params;
}

vector<double> PID::getErrors()
{
  return errors;
}

double PID::getNextAction()
{
  double result;
  if (scheduled_act.size()>0) {
    result = scheduled_act[0];
    scheduled_act.erase(scheduled_act.begin());
  } else {
    result = prev_action;
  }
  prev_action = result;
  return result;
}

double PID::getLastAction()
{
  double result;
  if (scheduled_act.size()>0) {
    result = scheduled_act.back();
  } else {
    result = prev_action;
  }
  return result;
}

int PID::getSchedActionCount()
{
  int result = scheduled_act.size();
  return result;
}

void PID::updateScheduledAction(double act)
{
  double prev_act = getLastAction();
  int steps = (int)fmod(fabs(act-prev_act), act_resolution) + 1;
  double pitch = (act-prev_act)/(double)steps;
  for (int i=0; i<steps; i++) {
    scheduled_act.push_back(prev_act + (double)(i+1)*pitch);
  }
}

double PID::run(double hi_bnd, double lo_bnd, double max_ch_rate, double& err)
{
    double result;
    double diff_cte;
    double allowed_change = fabs(hi_bnd - lo_bnd)*max_ch_rate;
    if (run_i < run_2n) {
      switch (run_state) {
        case 0:
          run_cte = prev_cte;;
          diff_cte = (run_cte - run_prev_cte)/prev_dt;
          run_int_cte += run_cte*prev_dt;
          run_prev_cte = run_cte;
          result = -run_p[0] * run_cte - run_p[1] * diff_cte - run_p[2] * run_int_cte;
          if (result < lo_bnd || result > hi_bnd || fabs(result-run_prev_result) > allowed_change ) {
            if (fabs(result-run_prev_result)> allowed_change) {
                if (result > run_prev_result) { result = run_prev_result + allowed_change; }
                if (result < run_prev_result) { result = run_prev_result - allowed_change; }
            }
            if (result < lo_bnd) { result = lo_bnd; }
            if (result > hi_bnd ) { result = hi_bnd; }
            run_error = std::numeric_limits<double>::infinity();
            err = run_error;
            run_stop = true;
          } 
          run_prev_result = result;
          
          run_state ++;
          break;

        case 1:
          result = run_prev_result;
          if (run_stop) {
            err = run_error;
            run_i = run_2n;
          } else if (run_i >= run_n) { 
            run_error += pow(run_cte, 2.0); 
          }
  
          run_state = 0;
          run_i ++;
          break;
      }
    } else {
        run_state = 0;
        run_running = false;
        err = (double)run_error/(double)run_n;
        result = run_prev_result;
    }
    return result;
}

void PID::UpdateError(double cte, double speed, double angle, double delta_t) {
  /**
   * TODO: Udate PID errors based on cte.
   */
  if (delta_t < 0.0001) { delta_t = 0.0001; }
  d_error = (cte - prev_cte)/delta_t;;
  i_error += cte*delta_t;
  p_error = cte;
  prev_cte = cte;
  errors = {p_error, i_error, d_error};
  prev_dt = delta_t;
  prev_speed = speed;
  prev_angle = angle;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double dp_sum = dp[0] + dp[1] + dp[2];
  double result = 0.0;
  bool   done = false;
  double up_mult = 1.01;
  double down_mult = 0.99;
  double allowed_change = fabs(hi_bound - lo_bound)*max_change_rate;

  if (dp_sum <= tolerance || !enable_twiddle) {
    result = -(params[0] * errors[0]) - (params[1] * errors[1]) - (params[2] * errors[2]);
    if (result < lo_bound || result > hi_bound || fabs(result-prev_action) > allowed_change ) {
      if (fabs(result-prev_action)> allowed_change) {
          if (result > prev_action) { result = prev_action + allowed_change; }
          if (result < prev_action) { result = prev_action - allowed_change; }
      }
      if (result < lo_bound) { result = lo_bound; }
      if (result > hi_bound ) { result = hi_bound; }
    }
  } else {
    do {
      switch (tw_state) {
        case 0:
          initRun(params, run_iteration);
          tw_state ++;
          done = false;
          break;

        case 1:
          if (run_running) {
            result = run(hi_bound, lo_bound, max_change_rate, tw_err);
            done = true;
          } else {
            best_err = tw_err;
            tw_state ++;
            done = false;
          }
          break;
         
        case 2:
          params[pIndex] += dp[pIndex]; 
          initRun(params, run_iteration);
          tw_state ++;
          done = false;
          break;

        case 3:
          if (run_running) {
            result = run(hi_bound, lo_bound, max_change_rate, tw_err);
            done = true;
          } else {
            done = false;
            tw_state ++;
          }
          break;

        case 4:
          if (tw_err < best_err) {
            best_err = tw_err;
            dp[pIndex] *= up_mult;
            tw_state = 2;
            done = false;
            pIndex = (++pIndex)%params.size();
          } else {
            params[pIndex] -= 2.0 * dp[pIndex];
            initRun(params, run_iteration);
            tw_state ++;
            done = false;
          }
          break;

        case 5:
          if (run_running) {
            result = run(hi_bound, lo_bound, max_change_rate, tw_err);
            done = true;
          } else {
            done = false;
            tw_state ++;
          }
          break;

        case 6:
          if (tw_err < best_err) {
            best_err = tw_err;
            dp[pIndex] *= up_mult;
          } else {
            params[pIndex] += dp[pIndex];
            dp[pIndex] *= down_mult;
          }
          tw_state = 2;
          done = false;
          pIndex = (++pIndex)%params.size();
          break;
      }
    } while (!done);
  }
  // std::cout << " id: " << id << " pa[0]: " << params[0] << " pa[1]: " << params[1] << " pa[1]: " << params[2] << " pi: " << pIndex << std::endl;
  // std::cout << " id: " << id << " dp_sum: " << dp_sum << " result: " << result << " speed: " << prev_speed << std::endl; 
  // std::cout << " id: " << id << " dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[1]: " << dp[2] << " tw_s: " << tw_state << " pi: " << pIndex << std::endl;
  // std::cout << "==> id: " << id << " result: " << result << " run_i " << run_i << " run_n " << run_n << std::endl;
  updateScheduledAction(result);
  return result; 
}



