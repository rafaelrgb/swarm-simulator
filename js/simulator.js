// Simulator variables
var isPaused = false;
var frequency = 100;
var dt = 1 / frequency;

// Canvas variables
var canvas = document.getElementById('canvas');
var treshold = 200;
var colors = [
  "#83984d",
  "#423f3f",
  "#f0b51f",
  "#ea700b",
  "#336b8b"
];
var paint = false;
var obstacleTool = false;
var goalTool = false;

// World variables
var robots = [];
var obstacles = [];
var goal = null;
var visionDistance = 100;
var environment = {
  'robotRadius': 0,
  'obstacleRadius': 0
}

// Debug variables
var lines = [];


// Simulation functions
if (typeof init !== "function") { 
  window.init = function(){};
}
if (typeof update !== "function") {
  window.update = function(){};
}

function reset() {
  robots = [];
  obstacles = [];
  lines = [];
  goal = null;
  init();
  drawCanvas();
}

function pauseResume() {
  if(!isPaused) {
    isPaused = true;
    $("#pause-btn").html("<i class=\"fas fa-play\"></i>");
  } else {
    isPaused = false;
    $("#pause-btn").html("<i class=\"fas fa-pause\"></i>");
  }
}


// Canvas related functions
function drawCanvas() {
  // Fill text fields with parameters values
  $('#nrobots').html(robots.length);

  if (canvas.getContext) {
    var ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw the robots
    for (var i = 0; i < robots.length; i++) {
      var r = (robots[i].radius) != 0 ? robots[i].radius : 5;
      ctx.beginPath();
      ctx.arc(robots[i].position.x, robots[i].position.y, r, 0, 2*Math.PI);
      ctx.fillStyle = colors[ i % 5 ];
      ctx.fill();
    }
    
    // Draw the obstacles
    for(var i = 0; i < obstacles.length; i++) {
      var r = (obstacles[i].radius) != 0 ? obstacles[i].radius : 5;
      ctx.beginPath();
      ctx.arc(obstacles[i].position.x, obstacles[i].position.y, r, 0, 2*Math.PI);
      ctx.fillStyle = "#000000";
      ctx.fill();
    }

    // Draw the goal
    if(goal !== null) {
      ctx.beginPath();
      ctx.arc(goal.x, goal.y, 5, 0, 2*Math.PI);
      ctx.fillStyle = "#FF0000";
      ctx.fill();
    }      

    // Draw the lines
    for(var l = 0; l < lines.length; l++) {
      ctx.beginPath();
      ctx.moveTo(lines[l].xOrig, lines[l].yOrig);
      ctx.lineTo(lines[l].xDest, lines[l].yDest);
      ctx.strokeStyle = lines[l].color;
      ctx.stroke();
    }
  }
}

$('#canvas').mousedown(function(e){
  if(obstacleTool || goalTool) {
    if(obstacleTool) {
      paint = true;
      addObstacle(e.pageX - this.offsetLeft, e.pageY - this.offsetTop, 0, 0, environment.obstacleRadius);
    }
    if(goalTool) {
      setGoal(e.pageX - this.offsetLeft, e.pageY - this.offsetTop);
    }    
    drawCanvas();
  }
});

$('#canvas').mousemove(function(e){
  if(obstacleTool && paint){
    addObstacle(e.pageX - this.offsetLeft, e.pageY - this.offsetTop, 0, 0, environment.obstacleRadius);
    drawCanvas();
  }
});

$('#canvas').mouseup(function(e){
  paint = false;
});

$('#canvas').mouseleave(function(e){
  paint = false;
});


// Functions related to the world
function addRobot(x, y, vX, vY, r) {
  robots.push({
    position: {
      x: x,
      y: y
    },
    velocity: {
      x: vX,
      y: vY
    },
    radius: (typeof r !== 'undefined') ? r : environment.robotRadius,
    neighbors: [],
    obstacles: []
  });
  drawCanvas();
}

function removeRobot() {
  robots.pop();
  drawCanvas();
}

function addObstacle(x, y, vX, vY, r) {
  obstacles.push({
    position: {
      x: x,
      y: y
    },
    velocity: {
      x: vX,
      y: vY
    },
    radius: (typeof r !== 'undefined') ? r : environment.obstacleRadius
  });
}

function addRandonRobot() {
  addRobot(
    Math.floor((Math.random() * (canvas.width - 2*treshold)) + treshold),
    Math.floor((Math.random() * (canvas.height - 2*treshold)) + treshold),
    Math.floor((Math.random() * 200) - 100),
    Math.floor((Math.random() * 200) - 100),
    environment.robotRadius
  );
}

function addRandonObstacle() {
  addObstacle(
    Math.floor((Math.random() * (canvas.width - 2*treshold)) + treshold),
    Math.floor((Math.random() * (canvas.height - 2*treshold)) + treshold),
    Math.floor((Math.random() * 200) - 100),
    Math.floor((Math.random() * 200) - 100),
    environment.obstacleRadius
  );
}

function setGoal(x, y) {
  goal = { x: x, y: y };
}


// Debug functions
function addLine(color, xOrig, yOrig, xDest, yDest) {
  lines.push({
    'color': color,
    'xOrig': xOrig,
    'yOrig': yOrig,
    'xDest': xDest,
    'yDest': yDest
  });
}


// Run the simulation
$(document).ready(function() {  
  
  // Make canvas fullscreen and register function to resize it when the browser is resized
  function resizeCanvas() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
  }
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas, false);

  // Add functionality to the buttons
  $('#add-robot').click(addRandonRobot);
  $('#remove-robot').click(removeRobot);
  $('#reset').click(reset);
  $('#pause-btn').click(pauseResume);
  $('#goal-tool-btn').click(function() {
    if(!$(this).hasClass('active')) {
      $(this).addClass('active');
      $('#obstacle-tool-btn').removeClass('active');
      goalTool = true;
      obstacleTool = false;
    } else {
      $(this).removeClass('active');
      goalTool = false;
    }
  });
  $('#obstacle-tool-btn').click(function() {
    if(!$(this).hasClass('active')) {
      $(this).addClass('active');
      $('#goal-tool-btn').removeClass('active');
      goalTool = false;
      obstacleTool = true;
    } else {
      $(this).removeClass('active');
      obstacleTool = false;
    }
  });

  // Init Simulation
  init();
  drawCanvas();

  // Simulation Loop
  setInterval(function() {
    if(!isPaused) {
      lines = [];

      // For each robot, detect its neighbors and obstacles
      for (var i = 0; i < robots.length; i++) {
        robot = robots[i];

        // Detect neighbors
        robot.neighbors = [];
        for (var n = 0; n < robots.length; n++) {
          if (n != i) {
            var dist = distance(robot.position, robots[n].position);
            if ((dist - robot.radius - robots[n].radius) < visionDistance || visionDistance == 0) {
              robot.neighbors.push(robots[n]);
            }
          }
        }

        // Detect obstacles
        robot.obstacles = [];
        for (var o = 0; o < obstacles.length; o++) {
          var dist = distance(robot.position, obstacles[o].position);
          if ((dist - robot.radius - obstacles[o].radius) < visionDistance || visionDistance == 0) {
            robot.obstacles.push(obstacles[o]);
          }
        }
      }

      update();

      // Update robots position    
      for (var i = 0; i < robots.length; i++) {
        robots[i].position = add(robots[i].position, scale(robots[i].velocity, dt));

        while (robots[i].position.x > canvas.width) robots[i].position.x -= canvas.width;
        while (robots[i].position.x < 0) robots[i].position.x += canvas.width;
        while (robots[i].position.y > canvas.height) robots[i].position.y -= canvas.height;
        while (robots[i].position.y < 0) robots[i].position.y += canvas.height;
      }

      // Update obstacles position
      for (var o = 0; o < obstacles.length; o++) {
        obstacles[o].position = add(obstacles[o].position, scale(obstacles[o].velocity, dt));

        while (obstacles[o].position.x > canvas.width) obstacles[o].position.x -= canvas.width;
        while (obstacles[o].position.x < 0) obstacles[o].position.x += canvas.width;
        while (obstacles[o].position.y > canvas.height) obstacles[o].position.y -= canvas.height;
        while (obstacles[o].position.y < 0) obstacles[o].position.y += canvas.height;
      }

      drawCanvas();
    }    
  }, 1000*dt);
  
})