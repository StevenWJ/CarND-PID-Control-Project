#ifndef PID_H
#define PID_H

#include <random>
#include <math.h>
#include <string>

using std::vector;
using std::string;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(string id_str, double Kp_, double Ki_, double Kd_, double hi_bnd, double lo_bnd, int tw_run_iter, double max_ch_rate, double act_res);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte, double speed, double prev_angle, double delta_t);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  vector<double> getParams();
  vector<double> getErrors();
  double getNextAction();
  double getLastAction();
  int    getSchedActionCount();
  void updateScheduledAction(double act);

  bool enable_twiddle;

 private:
  /**
   * Implementation of Twiddle
   * @output none
   */
  void initRun(const vector<double>& p, int n);
  double run(double hi_bnd, double lo_bnd, double max_ch_rate, double& err);

  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
  * Variables for Twiddle
  */
  
  int    pIndex;
  double best_err;
  double tw_err;
  double tolerance;
  int    tw_state;
  int    run_state;
  double hi_bound;
  double lo_bound;
  double prev_cte;
  double prev_dt;
  std::vector<double> params; 
  std::vector<double> errors; 
  std::vector<double> dp;
  std::vector<double> run_p;
  std::vector<double> scheduled_act;
  double run_error;
  int    run_n;
  int    run_2n;
  int    run_i;
  double run_prev_cte;
  double run_int_cte;
  double run_cte;
  bool   run_running;
  bool   run_stop;
  double run_prev_result;
  double prev_speed;
  double max_change_rate;
  double run_iteration;
  double prev_angle;
  double prev_action;
  double act_resolution;
  string id;
};

#endif  // PID_H
