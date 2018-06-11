//Quad Physics
var position = new THREE.Vector3(0, 0, 0);
var velocity = new THREE.Vector3(0, 0, 0);
var acceleration = new THREE.Vector3(0, 0, 0);

var angularPosition = new THREE.Quaternion();
var angularVelocity = new THREE.Vector3(0, 0, 0);
var angularAcceleration = new THREE.Vector3(0, 0, 0);

var motorOutputs = { b:0, c:0, d:0, e:0 };

var gravity = new THREE.Vector3(0, -9.81, 0);
var airDensity = 1.225;
var dragCoefficient = 1.0;
var area = 0.01;
var armLength = 100;//mm
var armAngle = 60;
var controlOutputs = { y:0, p:0, r:0, t:-gravity.y / 4 };

var startTime = new Date();
var dT = 0;
var angle = true;
var scale = 0.5;

var acroR = 10;
var acroT = 10;
var angleR = 2;
var angleT = 10;

var mass = 0.2;

var flightStyle = 0;

var xPos;
var yPos;

var yaw;

class PID {
  constructor(kp, ki, kd){
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.integral = 0;
    this.previousError = 0;
  }

  calculate(sp, pv, dT) {
    var p, i, d, error, errorOffset;

    error = sp - pv;
    this.integral += error * dT;
    errorOffset = (error - this.previousError) / dT;

    p = this.kp * error;
    i = this.ki * this.integral;
    d = this.kd * errorOffset;

    this.previousError = error;

    return p + i + d;
  };
}

class CDS {
  constructor(sC){
    this.sC = sC;
    this.velocity = 0.0;
    this.position = 0.0;
  }

  calculate(sp, dT) {
    var currToTarg = sp - this.position;
    var springForce = currToTarg * this.sC;
    var dampingForce = -this.velocity * 2.0 * Math.sqrt(this.sC);
    var force = springForce + dampingForce;

    this.velocity = this.velocity + force * dT;
    this.position = this.position + this.velocity * dT;

    return this.position;
  };
}

function Angle(e){
  angle = true;
  ClearControls();

  document.getElementById('AngleM').className = 'dropdown-item active';
  document.getElementById('AcrobM').className = 'dropdown-item';
}

function Acro(e){
  angle = false;
  ClearControls();

  document.getElementById('AngleM').className = 'dropdown-item';
  document.getElementById('AcrobM').className = 'dropdown-item active';
}

function PRStyle(e){
  flightStyle = 0;
  ClearControls();

  document.getElementById('PRS').className = 'dropdown-item active';
  document.getElementById('PYS').className = 'dropdown-item';
  document.getElementById('PTS').className = 'dropdown-item';
}

function PYStyle(e){
  flightStyle = 1;
  ClearControls();

  document.getElementById('PRS').className = 'dropdown-item';
  document.getElementById('PYS').className = 'dropdown-item active';
  document.getElementById('PTS').className = 'dropdown-item';
}

function PTStyle(e){
  flightStyle = 2;
  ClearControls();

  document.getElementById('PRS').className = 'dropdown-item';
  document.getElementById('PYS').className = 'dropdown-item';
  document.getElementById('PTS').className = 'dropdown-item active';
}

//b c
//e d

function Simulate() {
  var endTime = new Date();
  dT = (endTime - startTime) / 1000;
  startTime = endTime;

  if(flightStyle == 1){
    controlOutputs.y += yaw;
  }

  GetMotorOutputs();
  EstimateRotation();
  EstimatePosition();
}

var x, y;

function ClearControls(){
  position = new THREE.Vector3(0, 0, 0);
  velocity = new THREE.Vector3(0, 0, 0);
  acceleration = new THREE.Vector3(0, 0, 0);

  angularPosition = new THREE.Quaternion();
  angularVelocity = new THREE.Vector3(0, 0, 0);
  angularAcceleration = new THREE.Vector3(0, 0, 0);

  controlOutputs.y = 0;//z
  controlOutputs.p = 0;//x
  controlOutputs.r = 0;//y
  controlOutputs.t = -gravity.y / 4;

  GetMotorOutputs();

  quadGroup.rotation.x = 0;
  quadGroup.rotation.y = 0;
  quadGroup.rotation.z = Math.PI / 2;
}

function Reset(e){
  ClearControls();
}

function SetControls(e){
  xPos =  (e.clientX - window.innerWidth  / 2) * 1.4;
  yPos = -(e.clientY - window.innerHeight / 2) * 1.4;

  x = (e.clientX - window.innerWidth  / 2) / (window.innerWidth  / 2) * 0.6;//target angle
  y = (e.clientY - window.innerHeight / 2) / (window.innerHeight / 2) * 0.6;//target angle

  if (flightStyle == 0){//pitch roll
    controlOutputs.y = 0;
    controlOutputs.p = y;
    controlOutputs.r = x;
    controlOutputs.t = -gravity.y / 4;
  }
  else if (flightStyle == 1){//pitch yaw
    yaw = -x / 4.0;
    //controlOutputs.y;
    controlOutputs.p = 0;
    controlOutputs.r = y;
    controlOutputs.t = -gravity.y / 4;
  }
  else if (flightStyle == 2){//pitch throttle
    controlOutputs.y = x;
    controlOutputs.p = y;
    controlOutputs.r = 0;
    controlOutputs.t = -gravity.y / 4;
  }
}

var rotationPIDP = new PID(0.1, 0, 0.06);
var rotationPIDR = new PID(0.1, 0, 0.06);
var rotationPIDY = new PID(0.1, 0, 0.06);

function GetMotorOutputs(){
  if(angle){
    //difference between two quaternions
    var targetQ = new THREE.Quaternion().setFromEuler(new THREE.Euler(controlOutputs.p, controlOutputs.r, controlOutputs.y, 'ZYX'));

    console.log(targetQ);

    var qDif = new THREE.Quaternion(
      2.0 * (targetQ.x - angularPosition.x),
      2.0 * (targetQ.y - angularPosition.y),
      2.0 * (targetQ.z - angularPosition.z),
      2.0 * (targetQ.w - angularPosition.w)
    );

    var qMult = new THREE.Quaternion(
      -angularPosition.x,
      -angularPosition.y,
      -angularPosition.z,
       angularPosition.w
    );

    var quat = qDif.clone().multiply(qMult).clone();

    var pid = new THREE.Vector3(
      -rotationPIDP.calculate(0, quat.x / dT, dT),
      -rotationPIDR.calculate(0, quat.y / dT, dT),
      -rotationPIDY.calculate(0, quat.z / dT, dT)
    );

    motorOutputs.b = controlOutputs.t * angleT - pid.x * angleR + pid.y * angleR - pid.z * angleR;
    motorOutputs.c = controlOutputs.t * angleT - pid.x * angleR - pid.y * angleR + pid.z * angleR;
    motorOutputs.d = controlOutputs.t * angleT + pid.x * angleR - pid.y * angleR - pid.z * angleR;
    motorOutputs.e = controlOutputs.t * angleT + pid.x * angleR + pid.y * angleR + pid.z * angleR;
  }
  else{
    //acrobatics
    motorOutputs.b = controlOutputs.t * acroT - controlOutputs.p * acroR + controlOutputs.r * acroR - controlOutputs.y * acroR;
    motorOutputs.c = controlOutputs.t * acroT - controlOutputs.p * acroR - controlOutputs.r * acroR + controlOutputs.y * acroR;
    motorOutputs.d = controlOutputs.t * acroT + controlOutputs.p * acroR - controlOutputs.r * acroR - controlOutputs.y * acroR;
    motorOutputs.e = controlOutputs.t * acroT + controlOutputs.p * acroR + controlOutputs.r * acroR + controlOutputs.y * acroR;
  }

  //console.log(motorOutputs);
/*
  if(motorOutputs.b < 0){
    motorOutputs.b = 0;
  }
  if(motorOutputs.c < 0){
    motorOutputs.c = 0;
  }
  if(motorOutputs.d < 0){
    motorOutputs.d = 0;
  }
  if(motorOutputs.e < 0){
    motorOutputs.e = 0;
  }
  */
}

function EstimateRotation(){
  var torque = armLength * Math.sin((180 - armAngle) * Math.PI / 180);
  var dragForce = new THREE.Vector3(
    0.5 * airDensity * Math.pow(angularVelocity.x, 2) * dragCoefficient * area * Math.sign(angularVelocity.x),
    0.5 * airDensity * Math.pow(angularVelocity.y, 2) * dragCoefficient * area * Math.sign(angularVelocity.y),
    0.5 * airDensity * Math.pow(angularVelocity.z, 2) * dragCoefficient * area * Math.sign(angularVelocity.z)
  );//.applyQuaternion(angularPosition);

  angularAcceleration = new THREE.Vector3(
    ((motorOutputs.d + motorOutputs.e) - (motorOutputs.b + motorOutputs.c)) * torque,
    ((motorOutputs.b + motorOutputs.e) - (motorOutputs.c + motorOutputs.d)) * torque,
    ((motorOutputs.c + motorOutputs.e) - (motorOutputs.b + motorOutputs.d)) * torque
  );//.applyQuaternion(angularPosition);

  angularAcceleration.multiplyScalar(Math.PI / 180);
  angularVelocity.add(angularAcceleration.clone().multiplyScalar(dT).sub(dragForce.multiplyScalar(dT)));

  var angularQuatRate = new THREE.Quaternion(
    angularVelocity.x * 0.5 * dT,
    angularVelocity.y * 0.5 * dT,
    angularVelocity.z * 0.5 * dT,
    0
  );

  angularQuatRate.multiply(angularPosition);

  angularPosition = new THREE.Quaternion(
    angularPosition.x + angularQuatRate.x,
    angularPosition.y + angularQuatRate.y,
    angularPosition.z + angularQuatRate.z,
    angularPosition.w + angularQuatRate.w
  ).normalize();
}

function EstimatePosition(){
  var thrustSum = motorOutputs.b + motorOutputs.c + motorOutputs.d + motorOutputs.e;
  var dragForce = new THREE.Vector3(
    0.5 * airDensity * Math.pow(velocity.x, 2.0) * dragCoefficient * area * Math.sign(velocity.x),
    0.5 * airDensity * Math.pow(velocity.y, 2.0) * dragCoefficient * area * Math.sign(velocity.y),
    0.5 * airDensity * Math.pow(velocity.z, 2.0) * dragCoefficient * area * Math.sign(velocity.z)
  );

  acceleration = (new THREE.Vector3(0, 0, thrustSum)).applyQuaternion(angularPosition);//.clone().add(gravity);
  velocity.add(acceleration.clone().multiplyScalar(dT / mass)).sub(dragForce.multiplyScalar(dT));
  position.add(velocity.clone().multiplyScalar(dT / mass));

  if(position.x > (window.innerWidth / 1.5)){
    position.x = (window.innerWidth / 1.5);
    velocity.x = velocity.x * -0.25;
  }

  if(position.x < -(window.innerWidth / 1.5)){
    position.x = -(window.innerWidth / 1.5);
    velocity.x = velocity.x * -0.25;
  }

  if(position.y > (window.innerHeight / 1.5)){
    position.y = (window.innerHeight / 1.5);
    velocity.y = velocity.y * -0.25;
  }

  if(position.y < -(window.innerHeight / 1.5)){
    position.y = -(window.innerHeight / 1.5);
    velocity.y = velocity.y * -0.25;
  }
}

////////////////////////////////////////////////////////////////////////////////
//Quad Display
if ( ! Detector.webgl ) Detector.addGetWebGLMessage();

var container, stats, camera, cameraTarget, scene, renderer;
var quad, p1, p2, p3, p4, g, t;

var normalMat = new THREE.MeshNormalMaterial();
var geometry = new THREE.BoxGeometry( 1, 1, 1 );

var quadGroup = new THREE.Object3D();
var gateGroup = new THREE.Object3D();

quad = new THREE.Mesh( geometry, normalMat );
p1   = new THREE.Mesh( geometry, normalMat );
p2   = new THREE.Mesh( geometry, normalMat );
p3   = new THREE.Mesh( geometry, normalMat );
p4   = new THREE.Mesh( geometry, normalMat );

g    = new THREE.Mesh( geometry, normalMat );

t    = new THREE.Mesh( geometry, normalMat );

var cdsB = new CDS(100);
var cdsC = new CDS(100);
var cdsD = new CDS(100);
var cdsE = new CDS(100);

var b = 0.0, c = 0.0, d = 0.0, e = 0.0;

init();
animate();

var viewerDiv = $("viewer");

function init() {

  container = document.getElementById("viewer");

  camera = new THREE.PerspectiveCamera( 70, window.innerWidth / window.innerHeight, 1, 10000 );
  camera.position.set( 0, 0, 1000 );

  cameraTarget = new THREE.Vector3( 0, 0, 0 );

  scene = new THREE.Scene();

  var loader = new THREE.STLLoader();

  // Quadcopter mesh
  loader.load( '/js/FullQuadcopter.stl', function ( geometry ) {
    quad = new THREE.Mesh( geometry, normalMat );

    quad.position.set( 0, 0, 26.5 * scale );
    quad.rotation.set( 0, 0, 0 );
    quad.scale.set( scale, scale, scale );

    quad.castShadow = true;
    quad.receiveShadow = true;

    quadGroup.add(quad);
  });

  // Prop mesh
  loader.load( '/js/Prop.stl', function ( geometry ) {
    p1 = new THREE.Mesh( geometry, normalMat );

    p1.position.set( 88.142 * scale, 70.642 * scale, 0 );
    p1.rotation.set( 0, 0, 0 );
    p1.scale.set( scale, scale, scale );

    p1.castShadow = true;
    p1.receiveShadow = true;

    quadGroup.add(p1);
  });
  loader.load( '/js/Prop.stl', function ( geometry ) {
    p2 = new THREE.Mesh( geometry, normalMat );

    p2.position.set( -88.142 * scale, 70.642 * scale, 0 );
    p2.rotation.set( 0, 0, 0 );
    p2.scale.set( -scale, scale, scale );

    p2.castShadow = true;
    p2.receiveShadow = true;

    quadGroup.add(p2);
  });
  loader.load( '/js/Prop.stl', function ( geometry ) {
    p3 = new THREE.Mesh( geometry, normalMat );

    p3.position.set( -88.142 * scale, -70.642 * scale, 0 );
    p3.rotation.set( 0, 0, 0 );
    p3.scale.set( -scale, -scale, scale );

    p3.castShadow = true;
    p3.receiveShadow = true;

    quadGroup.add(p3);
  });
  loader.load( '/js/Prop.stl', function ( geometry ) {
    p4 = new THREE.Mesh( geometry, normalMat );

    p4.position.set( 88.142 * scale, -70.642 * scale, 0 );
    p4.rotation.set( 0, 0, 0 );
    p4.scale.set( scale, -scale, scale );

    p4.castShadow = true;
    p4.receiveShadow = true;

    quadGroup.add(p4);
  });

  loader.load( '/js/gate.stl', function ( geometry ) {
    g = new THREE.Mesh( geometry, normalMat );

    g.position.set( 0, window.innerHeight / 2, 0 );
    g.rotation.set( 0, 0, Math.PI / 2 );
    g.scale.set( 0.05, 0.05, 0.05 );

    g.castShadow = true;
    g.receiveShadow = true;

    gateGroup.add(g);
  });
  loader.load( '/js/gate.stl', function ( geometry ) {
    g = new THREE.Mesh( geometry, normalMat );

    g.position.set( 0, -window.innerHeight / 2, 0 );
    g.rotation.set( 0, 0, Math.PI / 2 );
    g.scale.set( 0.05, 0.05, 0.05 );

    g.castShadow = true;
    g.receiveShadow = true;

    gateGroup.add(g);
  });
  loader.load( '/js/gate.stl', function ( geometry ) {
    g = new THREE.Mesh( geometry, normalMat );

    g.position.set( window.innerWidth / 2, 0, 0 );
    g.rotation.set( 0, 0, 0 );
    g.scale.set( 0.05, 0.05, 0.05 );

    g.castShadow = true;
    g.receiveShadow = true;

    gateGroup.add(g);
  });
  loader.load( '/js/gate.stl', function ( geometry ) {
    g = new THREE.Mesh( geometry, normalMat );

    g.position.set( -window.innerWidth / 2, 0, 0 );
    g.rotation.set( 0, 0, 0 );
    g.scale.set( 0.05, 0.05, 0.05 );

    g.castShadow = true;
    g.receiveShadow = true;

    gateGroup.add(g);
  });

  loader.load( '/js/Target.stl', function ( geometry ) {
    t = new THREE.Mesh( geometry, normalMat );

    t.position.set( 0, 0, 0 );
    t.rotation.set( 0, 0, 0 );
    t.scale.set( 0.1, 0.1, 0.1 );

    t.castShadow = true;
    t.receiveShadow = true;

    scene.add(t);
  });

  scene.add(gateGroup);
  scene.add(quadGroup);

  quadGroup.rotation.z = Math.PI / 2;

  // Lights
  scene.add( new THREE.HemisphereLight( 0x333333, 0x111122 ) );
  scene.background = new THREE.Color( 0x444444 );

  // renderer
  renderer = new THREE.WebGLRenderer( { antialias: true } );
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );

  container.appendChild( renderer.domElement );

  window.addEventListener( 'resize', onWindowResize, false );
}

function onWindowResize() {
	camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize( window.innerWidth, window.innerHeight );
}

function animate() {
  requestAnimationFrame( animate );

  render();
}


function render() {
  Simulate();

  var timer = Date.now() * 0.0025;

  //quadGroup.rotation.set( 0, 0.707, Math.sin( timer ) );


  var cb = (cdsB.calculate( motorOutputs.b, dT) - 24.0) / 2.0;
  var cc = (cdsC.calculate( motorOutputs.c, dT) - 24.0) / 2.0;
  var cd = (cdsD.calculate( motorOutputs.d, dT) - 24.0) / 2.0;
  var ce = (cdsE.calculate( motorOutputs.e, dT) - 24.0) / 2.0;

  b += (cb * (180.0 / Math.PI) % 360) * (Math.PI / 180.0);
  c += (cc * (180.0 / Math.PI) % 360) * (Math.PI / 180.0);
  d += (cd * (180.0 / Math.PI) % 360) * (Math.PI / 180.0);
  e += (ce * (180.0 / Math.PI) % 360) * (Math.PI / 180.0);

  p1.rotation.set(0, 0, -b);
  p2.rotation.set(0, 0,  c);
  p3.rotation.set(0, 0, -d);
  p4.rotation.set(0, 0,  e);

  t.position.copy(new THREE.Vector3(xPos, yPos, 0));

  quadGroup.quaternion.copy(angularPosition);
  quadGroup.position.copy(new THREE.Vector3(position.x, position.y, 0));

  //camera.position.set( Math.sin(timer) * 4, 3.14, Math.cos( timer ) * 4 );

  //console.log(angularPosition);

  camera.lookAt( cameraTarget );
  renderer.render( scene, camera );
}
