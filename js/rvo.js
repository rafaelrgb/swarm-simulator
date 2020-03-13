// Angles in the range (-pi, pi]
function in_between(theta_right, theta_dif, theta_left) {
  if(Math.abs(theta_right - theta_left) <= Math.PI) {
    if(theta_right <= theta_dif && theta_dif <= theta_left) {
      return true;
    } else {
      return false;
    }
  } else {
    if (theta_left < 0 && theta_right > 0) {
      theta_left += 2 * Math.PI;
      if(theta_dif < 0) {
        theta_dif += 2 * Math.PI;
      }
      if(theta_right <= theta_dif && theta_dif <= theta_left) {
        return true;
      } else {
        return false;
      }
    }
    if (theta_left > 0 && theta_right < 0) {
      theta_right += 2 * Math.PI;
      if(theta_dif < 0) {
        theta_dif += 2 * Math.PI;
      }
      if(theta_right <= theta_dif && theta_dif <= theta_left) {
        return true;
      } else {
        return false;
      }
    }
  }
}