// Vector related functions
function distance(v1, v2) {
  return Math.sqrt(Math.pow((v1.x - v2.x), 2) + Math.pow((v1.y - v2.y), 2));
}

function add(v1, v2) {
  return { x: v1.x + v2.x, y: v1.y + v2.y };
}

function sub(v1, v2) {
  return { x: v1.x - v2.x, y: v1.y - v2.y };
}

function length(v) {
  return Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2));
}

function normalize(v) {
  var l = length(v);
  return { x: v.x / l, y: v.y / l };
}

function scale(v, s) {
  return { x: s * v.x, y: s * v.y };
}

function det(v1, v2) {
  return v1.x * v2.y - v1.y * v2.x;
}

function normal(v1, v2) {
  return normalize({ x: v2.y - v1.y, y: v1.x - v2.x });
}

function dot(v1, v2) {
  return v1.x * v2.x + v1.y * v2.y;
}