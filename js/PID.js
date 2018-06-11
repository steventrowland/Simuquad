function PID(kp, ki, kd){
  this.kp = kp;
  this.ki = ki;
  this.kd = kd;
  this.integral = 0;
  this.previousError = 0;

  this.Calculate = function(sp, pv, dT) {
    var p, i, d, error, errorOffset;

    error = sp - pv;
    integral += error * dT;
    errorOffset = (error - previousError) / dT;

    p = kp * error;
    i = ki * integral;
    d = kd * errorOffset;

    previousError = error;
    
    return p + i + d;
  }
}
