// Flocking variables
r1 = 1.0;
r2 = 1.0;
r3 = 0.9;
r4 = 1.0;
r5 = 1.0;
var maxVelocity = 400;
var safetyRadius = 100;

// Submit parameters
// function submitParameters() {
//   r1 = $('#r1').val();
//   r2 = $('#r2').val();
//   r3 = $('#r3').val();
//   r4 = $('#r4').val();
//   visionDistance = $('#vision-distance').val();
//   safetyRadius = $('#safety-radius').val();
// }


function init() {
  var initialRobots = 5;
  var initialObstacles = 0;

  for (var i = 1; i <= initialRobots; i++) {
    addRandonRobot()
  }
  for(var i = 1; i <= initialObstacles; i++) {
    addRandonObstacle()
  }
}

function update() {
  // For each robot, calculate the behaviors
  for (var i = 0; i < robots.length; i++) {
    robot = robots[i];
    neighbors = robot.neighbors;
    detectedObstacles = robot.obstacles;

    // Rule1: Separation
    var v1 = { x: 0.0, y: 0.0 };
    if (neighbors.length > 0) {
      for (var n = 0; n < neighbors.length; n++) {
        neighbor = neighbors[n];
        repulsion = sub(robot.position, neighbor.position);
        repulsion = normalize(repulsion);        
        dist = distance(robot.position, neighbor.position);
        if(dist > 0.0) {
          repulsion = scale(repulsion, safetyRadius * safetyRadius / dist);
        }
        v1 = add(v1, repulsion);
      }
      //v1 = scale(v1, 1 / neighbors.length);
    }

    // Rule2: Cohesion
    var v2 = { x: 0.0, y: 0.0 };
    var cm = { x: 0.0, y: 0.0 };
    if (neighbors.length > 0) {
      for (var n = 0; n < neighbors.length; n++) {
        neighbor = neighbors[n];
        cm = add(cm, neighbor.position);
      }
      cm = scale(cm, 1 / neighbors.length);
      v2 = sub(cm, robot.position);
    }

    // Rule3: Alignment
    v3 = { x: 0.0, y: 0.0 }
    if (neighbors.length > 0) {
      for (var n = 0; n < neighbors.length; n++) {
        v3 = add(v3, neighbors[n].velocity);
      }
      v3 = scale(v3, 1/neighbors.length);
    }

    // Rule4: Migration
    var v4 = (goal !== null) ? sub(goal, robot.position) : { x: 0.0, y: 0.0 };


    // Rule5: Obstacle avoidance
    var v5 = { x: 0.0, y: 0.0 };
    if (detectedObstacles.length > 0) {
      for (var o = 0; o < detectedObstacles.length; o++) {
        obstacle = detectedObstacles[o];
        repulsion = sub(robot.position, obstacle.position);
        repulsion = normalize(repulsion);          
        dist = distance(robot.position, obstacle.position);
        if(dist > 0.0) {
          repulsion = scale(repulsion, safetyRadius * safetyRadius / dist);
        }
        v5 = add(v5, repulsion);
      }
      //v5 = scale(v5, 1 / detectedObstacles.length);
    }
    
    //addLine(colors[i], robot.position.x, robot.position.y, v1.x + robot.position.x, v1.y + robot.position.y);
    //addLine(colors[i], robot.position.x, robot.position.y, v2.x + robot.position.x, v2.y + robot.position.y);
    //addLine(colors[i], robot.position.x, robot.position.y, v3.x + robot.position.x, v3.y + robot.position.y);
    //addLine(colors[i], robot.position.x, robot.position.y, v4.x + robot.position.x, v4.y + robot.position.y);
    //addLine(colors[i], robot.position.x, robot.position.y, v5.x + robot.position.x, v5.y + robot.position.y);

    robot.velocity.x = r1 * v1.x + r2 * v2.x + r3 * v3.x + r4 * v4.x + r5 * v5.x;
    robot.velocity.y = r1 * v1.y + r2 * v2.y + r3 * v3.y + r4 * v4.y + r5 * v5.y;

    // Check if robot's velocity exceeds maximum and resize it
    if(length(robot.velocity) > maxVelocity) {
      robot.velocity = scale(normalize(robot.velocity), maxVelocity);
    }
  }
}



// $(document).ready(function() {
//   $('#submit-parameters').click(submitParameters);
//   $('#r1').val(r1);
//   $('#r2').val(r2);
//   $('#r3').val(r3);
//   $('#r4').val(r4);
//   $('#vision-distance').val(visionDistance);
//   $('#safety-radius').val(safetyRadius);
// });