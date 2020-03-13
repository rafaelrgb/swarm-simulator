// Set the environment
environment.robotRadius = 50;
environment.obstacleRadius = 50;
visionDistance = 400;


robotWaypoints = [];
obstacleWaypoints = [];

bound = 2;
safetyRadius = 20;
robotVelocity = 300;
obstacleVelocity = 300;

experiment = 2;

uncertaintyOffset = 0.0;


// TODO: renomear robotVelocity para maxSpeed
// TODO: renomear desiredVelocity para preferedVelocity
// TODO: descobrir o que é esse uncertaintyOffset





// Array de vos vazio - vamos criar um VO para cada neighbor
// O VO tem o seguinte formato:

// vo = {
//   Vector2 'apex_': The position of the apex of the hybrid reciprocal velocity obstacle.,
//   Vector2 'side1': The direction of the first side of the hybrid reciprocal velocity obstacle.,
//   Vector2 'side2': The direction of the second side of the hybrid reciprocal velocity obstacle.
// }


// Iterar pelos neighbors. Para cada neighbor, construir o VO correspondente.

// Calcular o angulo entre os dois robôs, ou seja, o angulo entre o eixo x e o vetor distancia entre os robôs.
// No buzz isso é chamado de azimuth.
// angle = atan(neighbor.p.y - me.p.y, neighbor.p.x - me.p.x)

// Calcular o angulo de abertura do VO.
// openingAngle = asin(neighbor.radius + me.radius, distance)

// Calcular o side1 e o side2. Ambos são vetores unitários na direção dos limites do VO
// side1 = Vector2(cos(angle - openingAngle), sin(angle - openingAngle))
// side2 = Vector2(cos(angle + openingAngle), sin(angle + openingAngle))









// function in_between(theta_right, theta_dif, theta_left) {
//   if(theta_right <= theta_dif && theta_dif <= theta_left) {
//     return true;
//   } else {
//     return false;
//   }
// }

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



var invTimeHorizonObst_ = 2.0;





// VO  magic happens here
function ORCA(robot, desiredVelocity) {



  orcaLines = [];


  invTimeHorizonObst = 1.0 / invTimeHorizonObst_;






  const float invTimeHorizonObst = 1.0f / timeHorizonObst_;

  const size_t numObstLines = orcaLines_.size();

		const float invTimeHorizon = 1.0f / timeHorizon_;

		/* Create agent ORCA lines. */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
			const Agent *const other = agentNeighbors_[i].second;

			const Vector2 relativePosition = other->position_ - position_;
			const Vector2 relativeVelocity = velocity_ - other->velocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);

			Line line;
			Vector2 u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);

				const float dotProduct1 = w * relativePosition;

				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;

					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}

					const float dotProduct2 = relativeVelocity * line.direction;

					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / sim_->timeStep_;

				/* Vector from cutoff center to relative velocity. */
				const Vector2 w = relativeVelocity - invTimeStep * relativePosition;

				const float wLength = abs(w);
				const Vector2 unitW = w / wLength;

				line.direction = Vector2(unitW.y(), -unitW.x());
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}

			line.point = velocity_ + 0.5f * u;
			orcaLines_.push_back(line);
		}

		size_t lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, newVelocity_);

		if (lineFail < orcaLines_.size()) {
			linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, newVelocity_);
		}





  var final_V = desiredVelocity;
  var collision = false;
  var suitable_V = [];

  var velocityObstacles = [];

  var velocityObstacle = {
    apex: { x: 0.0, y: 0.0 },
    side1: { x: 0.0, y: 0.0 },
    side2: { x: 0.0, y: 0.0 }
  }

  // for(var o = 0; o < robot.obstacles.length; o++) {
  //   var obstacle = robot.obstacles[o];
  //   var rArB = robot.radius + obstacle.radius + safetyRadius;
  
  //   theta_BA = Math.atan2(obstacle.position.y - robot.position.y, obstacle.position.x - robot.position.x);
  
  //   dist_BA = distance(obstacle.position, robot.position);
  //   if(rArB > dist_BA) {
  //     dist_BA = rArB;
  //   }
  //   theta_BA_ort = Math.asin(rArB / dist_BA);
  //   theta_BA_left = theta_BA - theta_BA_ort;
  //   bound_left = [ Math.cos(theta_BA_left), Math.sin(theta_BA_left) ];
  //   theta_BA_right = theta_BA + theta_BA_ort;
  //   bound_right = [ Math.cos(theta_BA_right), Math.sin(theta_BA_right) ];

  //   VO_all.push([obstacle.velocity, theta_BA_left, theta_BA_right]);

  //   // Detect collision
  //   // var vAB = sub(desiredVelocity, obstacle.velocity);
  //   // if(in_between(theta_BA_left, Math.atan2(vAB.y, vAB.x), theta_BA_right)) {
  //   //   collision = true;
  //   // }
  
  //   // addLine('#bcbcbc', robot.position.x, robot.position.y, 1000 * Math.cos(theta_BA) + robot.position.x, 1000 * Math.sin(theta_BA) + robot.position.y);
  //   // addLine('#ff0000', robot.position.x, robot.position.y, 1000 * bound_left[0] + robot.position.x, 1000 * bound_left[1] + robot.position.y);
  //   // addLine('#0000ff', robot.position.x, robot.position.y, 1000 * bound_right[0] + robot.position.x, 1000 * bound_right[1] + robot.position.y);
  // }

  for (var n = 0; n < robot.neighbors.length; n++) {
    var neighbor = robot.neighbors[n];
    var rArB = robot.radius + neighbor.radius + safetyRadius;
    var dist = distance(neighbor.position, robot.position);

    if (dist > rArB) {
      var angle = Math.atan2(neighbor.position.y - robot.position.y, neighbor.position.x - robot.position.x);
      var openingAngle = Math.asin(rArB / dist);

      velocityObstacle.side1 = { x: Math.cos(angle - openingAngle), y: Math.sin(angle - openingAngle) };
      velocityObstacle.side2 = { x: Math.cos(angle + openingAngle), y: Math.sin(angle + openingAngle) };

      var d = 2.0 * Math.sin(openingAngle) * Math.cos(openingAngle);

      if (det(sub(neighbor.position, robot.position), sub(neighbor.velocity, desiredVelocity)) > 0.0) {
        var s = 0.5 * det(sub(robot.velocity, neighbor.velocity), velocityObstacle.side2) / d;

        velocityObstacle.apex = sub(add(neighbor.velocity, scale(velocityObstacle.side1, s)), scale(uncertaintyOffset * dist / rArB, normalize(sub(neighbor.position, robot.position))));
      } else {
        var s = 0.5 * det(sub(robot.velocity, neighbor.velocity), velocityObstacle.side1) / d;

        velocityObstacle.apex = sub(add(neighbor.velocity, scale(velocityObstacle.side2, s)), scale(uncertaintyOffset * dist / rArB, normalize(sub(neighbor.position, robot.position))));
      }

      velocityObstacles.push(velocityObstacle);
    } else {
      velocityObstacle.apex = sub(scale(5.0, add(neighbor.velocity, robot.velocity)), scale(uncertaintyOffset + 5.0 * (rArB - dist) / dt, normalize(sub(neighbor.position, robot.position))));
      velocityObstacle.side1 = normal(robot.position, neighbor.position);
      velocityObstacle.side2 = sub({ x: 0.0, y: 0.0 }, velocityObstacle.side1);
      velocityObstacles.push(velocityObstacle);
    }

    // addLine('#bcbcbc', robot.position.x, robot.position.y, 1000 * Math.cos(angle) + robot.position.x, 1000 * Math.sin(angle) + robot.position.y);
    // addLine('#ff0000', robot.position.x, robot.position.y, 1000 * velocityObstacle.side1.x + robot.position.x, 1000 * velocityObstacle.side1.y + robot.position.y);
    // addLine('#0000ff', robot.position.x, robot.position.y, 1000 * velocityObstacle.side2.x + robot.position.x, 1000 * velocityObstacle.side2.y + robot.position.y);



  
    // theta_BA = Math.atan2(neighbor.position.y - robot.position.y, neighbor.position.x - robot.position.x);
  
    // dist_BA = distance(neighbor.position, robot.position);
    // if(rArB > dist_BA) {
    //   dist_BA = rArB;
    // }
    // theta_BA_ort = Math.asin(rArB / dist_BA);
    // theta_BA_left = theta_BA - theta_BA_ort;
    // bound_left = [ Math.cos(theta_BA_left), Math.sin(theta_BA_left) ];
    // theta_BA_right = theta_BA + theta_BA_ort;
    // bound_right = [ Math.cos(theta_BA_right), Math.sin(theta_BA_right) ];

    // VO_all.push([neighbor.velocity, theta_BA_left, theta_BA_right]);

    // Detect collision
    // var vAB = sub(desiredVelocity, neighbor.velocity);
    // if(in_between(theta_BA_left, Math.atan2(vAB.y, vAB.x), theta_BA_right)) {
    //   collision = true;
    // }
  
    // addLine('#bcbcbc', robot.position.x, robot.position.y, 1000 * Math.cos(theta_BA) + robot.position.x, 1000 * Math.sin(theta_BA) + robot.position.y);
    // addLine('#ff0000', robot.position.x, robot.position.y, 1000 * bound_left[0] + robot.position.x, 1000 * bound_left[1] + robot.position.y);
    // addLine('#0000ff', robot.position.x, robot.position.y, 1000 * bound_right[0] + robot.position.x, 1000 * bound_right[1] + robot.position.y);
  }



  var candidates = [];

  var candidate = {
    position: { x: 0.0, y: 0.0 },
    velocityObstacle1: 999999,
    velocityObstacle2: 999999
  }

  if(length(desiredVelocity) < robotVelocity) {
    candidate.position = desiredVelocity;
  } else {
    candidate.position = scale(normalize(desiredVelocity), robotVelocity);
  }

  candidates.push(candidate);

  for (var i = 0; i < velocityObstacles.length; i++) {
    candidate.velocityObstacle1 = i;
    candidate.velocityObstacle2 = i;

    var dotProduct1 = dot(sub(desiredVelocity, velocityObstacles[i].apex), velocityObstacles[i].side1);
    var dotProduct2 = dot(sub(desiredVelocity, velocityObstacles[i].apex), velocityObstacles[i].side2);

    if (dotProduct1 > 0.0 && det(velocityObstacles[i].side1, sub(desiredVelocity, velocityObstacles[i].apex)) > 0.0) {
      candidate.position = add(velocityObstacles[i].apex, scale(dotProduct1, velocityObstacles[i].side1));

      if (length(candidate.position) < robotVelocity) {
        candidates.push(candidate);
      }
    }

    if (dotProduct2 > 0.0 && det(velocityObstacles[i].side2, sub(desiredVelocity, velocityObstacles[i].apex)) < 0.0) {
      candidate.position = add(velocityObstacles[i].apex, scale(dotProduct2, velocityObstacles[i].side2));

      if (length(candidate.position) < robotVelocity) {
        candidates.push(candidate);
      }
    }
  }

    

  for (var j = 0; j < velocityObstacles.length; j++) {
    candidate.velocityObstacle1 = 999999;
    candidate.velocityObstacle2 = j;

    var discriminant = robotVelocity * robotVelocity - Math.pow(det(velocityObstacles[j].apex, velocityObstacles[j].side1), 2);

    if (discriminant > 0.0) {
      var t1 = -dot(velocityObstacles[j].apex, velocityObstacles[j].side1) + Math.sqrt(discriminant);
      var t2 = -dot(velocityObstacles[j].apex, velocityObstacles[j].side1) - Math.sqrt(discriminant);

      if (t1 >= 0.0) {
        candidate.position = add(velocityObstacles[j].apex, scale(t1, velocityObstacles[j].side1));
        candidates.push(candidate);
      }

      if (t2 >= 0.0) {
        candidate.position = add(velocityObstacles[j].apex, scale(t2, velocityObstacles[j].side1));
        candidates.push(candidate);
      }
    }

    discriminant = robotVelocity * robotVelocity - Math.pow(det(velocityObstacles[j].apex, velocityObstacles[j].side2), 2);

    if (discriminant > 0.0) {
      var t1 = -dot(velocityObstacles[j].apex, velocityObstacles[j].side2) + Math.sqrt(discriminant);
      var t2 = -dot(velocityObstacles[j].apex, velocityObstacles[j].side2) - Math.sqrt(discriminant);

      if (t1 >= 0.0) {
        candidate.position = add(velocityObstacles[j].apex, scale(t1, velocityObstacles[j].side2));
        candidates.push(candidate);
      }

      if (t2 >= 0.0) {
        candidate.position = add(velocityObstacles[j].apex, scale(t2, velocityObstacles[j].side2));
        candidates.push(candidate);
      }
    }
  }




  for (var i = 0; i < velocityObstacles.length; i++) {
    for (var j = 0; j < velocityObstacles.length; j++) {
      candidate.velocityObstacle1 = i;
      candidate.velocityObstacle2 = j;

      var d = det(velocityObstacles[i].side1, velocityObstacles[j].side1);

      if (d != 0.0) {
        var s = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[j].side1) / d;
        var t = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[i].side1) / d;

        if (s >= 0.0 && t >= 0.0) {
          candidate.position = add(velocityObstacles[i].apex, scale(s, velocityObstacles[i].side1));

          if (length(candidate.position) < robotVelocity) {
            candidates.push(candidate);
          }
        }
      }

      d = det(velocityObstacles[i].side2, velocityObstacles[j].side1);

      if (d != 0.0) {
        var s = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[j].side1) / d;
        var t = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[i].side2) / d;

        if (s >= 0.0 && t >= 0.0) {
          candidate.position = add(velocityObstacles[i].apex, scale(s, velocityObstacles[i].side2));

          if (length(candidate.position) < robotVelocity) {
            candidates.push(candidate);
          }
        }
      }

      d = det(velocityObstacles[i].side1, velocityObstacles[j].side2);

      if (d != 0.0) {
        var s = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[j].side2) / d;
        var t = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[i].side1) / d;

        if (s >= 0.0 && t >= 0.0) {
          candidate.position = add(velocityObstacles[i].apex, scale(s, velocityObstacles[i].side1));

          if (length(candidate.position) < robotVelocity) {
            candidates.push(candidate);
          }
        }
      }

      d = det(velocityObstacles[i].side2, velocityObstacles[j].side2);

      if (d != 0.0) {
        var s = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[j].side2) / d;
        var t = det(sub(velocityObstacles[j].apex, velocityObstacles[i].apex), velocityObstacles[i].side2) / d;

        if (s >= 0.0 && t >= 0.0) {
          candidate.position = add(velocityObstacles[i].apex, scale(s, velocityObstacles[i].side2));

          if (length(candidate.position) < robotVelocity) {
            candidates.push(candidate);
          }
        }
      }
    }
  }


  var optimal = -1;

  for (var c = 0; c < candidates.length; c ++) {
    candidate = candidates[c];
    var valid = true;

    for (var j = 0; j < velocityObstacles.length; j++) {
      if (j != candidate.velocityObstacle1 && j != candidate.velocityObstacle2 && det(velocityObstacles[j].side2, sub(candidate.position, velocityObstacles[j].apex)) < 0.0 && det(velocityObstacles[j].side1, sub(candidate.position, velocityObstacles[j].apex)) < 0.0) {
        valid = false;

        if (j > optimal) {
          optimal = j;
          final_V = candidate.position;
        }

        break;
      }
    }

    if (valid) {
      final_V = candidate.position;
      break;
    }
  }


  // Calculate suitable velocities
  // for(var th = 0; th < 2 * Math.PI; th += 0.1) {
  //   for(var rad = 0.02; rad <= length(desiredVelocity) + 0.02; rad += (length(desiredVelocity) / 5.0)) {
  //     var suit = true;
  //     var new_V = { x: rad * Math.cos(th), y: rad * Math.sin(th) };
      
  //     for(var vo = 0; vo < VO_all.length; vo++) {
  //       //var vAB = sub(new_V, VO_all[vo][0]);
  //       var vAB = sub(new_V, scale(add(desiredVelocity, VO_all[vo][0]), 1/2));
  //       if(in_between(VO_all[vo][1], Math.atan2(vAB.y, vAB.x), VO_all[vo][2])) {
  //         suit = false;
  //       }
  //     }
    
  //     if(suit) suitable_V.push(new_V);  
  //   }
  // }

  // if(suitable_V.length > 0) {
  //   var min_dist = 99999;
  //   for(var v = 0; v < suitable_V.length; v++) {
  //     if(distance(suitable_V[v], desiredVelocity) < min_dist) {
  //       min_dist = distance(suitable_V[v], desiredVelocity);
  //       final_V = suitable_V[v];
  //     }
  //   }
  // } else {
  //   final_V = { x: 0, y: 0 };
  // }

  return final_V;
}





function init() {

  if(experiment == 1) {

    addRobot(400, 400, 0, 0);
    addObstacle(600, 100, 0, 0);
    addObstacle(900, 700, 0, 0);
    addObstacle(1200, 100, 0, 0);

    robotWaypoints.push([
      { x: 1600, y: 400 },
      { x: 400, y: 400 }
    ]);

    obstacleWaypoints = [
      [
        { x: 900, y: 700 },
        { x: 600, y: 100 }
      ],
      [
        { x: 1200, y: 100 },
        { x: 900, y: 700 }
      ],
      [
        { x: 1500, y: 700 },
        { x: 1200, y: 100 }
      ]
    ];

  } else if(experiment == 2) {

    addRobot(600, 200, 0, 0);
    addRobot(1200, 200, 0, 0);
    addRobot(1200, 800, 0, 0);
    addRobot(600, 800, 0, 0);

    robotWaypoints = [
      [
        { x: 1200, y: 800 },
        { x: 600, y: 200 }
      ],
      [
        { x: 600, y: 800 },
        { x: 1200, y: 200 }
      ],
      [
        { x: 600, y: 200 },
        { x: 1200, y: 800 }
      ],
      [
        { x: 1200, y: 200 },
        { x: 600, y: 800 }
      ]
    ];

  } else if(experiment == 3) {

    for(var i = 0; i < 5; i++ ) {
      for(var j = 0; j < 5; j++) {
        addRandonRobot();
        robotWaypoints.push([
          { 
            x: 400 + 200 * i, 
            y: 200 + 150 * j 
          }
        ]);
      }
    }

  } else if(experiment == 4) {

    addRobot(400, 200, 0, 0);
    addRobot(400, 700, 0, 0);

    robotWaypoints.push([
      { x: 1500, y: 700 },
      { x: 400, y: 200 }
    ]);

    robotWaypoints.push([
      { x: 1500, y: 200 },
      { x: 400, y: 700 }
    ]);

  }
  

  for(var i = 0; i < robots.length; i++) {
    robots[i].wp = 0;
  }

  for(var o = 0; o < obstacles.length; o++) {
    obstacles[o].wp = 0;
  }
}


function update() {

  // Drive the robots
  for(var i = 0; i < robots.length; i++) {
    var robot = robots[i];
    var goal = robotWaypoints[i][robot.wp];
    var collision = false;

    var vector_to_goal = sub(goal, robot.position);
    var dist_to_goal = length(vector_to_goal);

    if(dist_to_goal >= bound) {
      desiredVelocity = (length(vector_to_goal) > robotVelocity) ? scale(normalize(vector_to_goal), robotVelocity) : vector_to_goal;
      robot.velocity = ORCA(robot, desiredVelocity);
    } else {
      robot.wp = (robot.wp + 1) % robotWaypoints[i].length;
    } 
  }

  // Drive the obstacles
  for(var o = 0; o < obstacles.length; o++) {
    var obstacle = obstacles[o];
    var goal = obstacleWaypoints[o][obstacle.wp];

    var vector_to_goal = sub(goal, obstacle.position);
    var dist_to_goal = length(vector_to_goal);

    if(dist_to_goal >= bound) {
      obstacle.velocity = scale(normalize(vector_to_goal), obstacleVelocity);
    } else {
      obstacle.wp = (obstacle.wp + 1) % obstacleWaypoints[o].length;
    } 
  }

}