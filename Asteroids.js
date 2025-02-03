// global variables for the player
let playerX;
let playerY;
let playerSpeed = 5;

// arrays to hold other game objects
let asteroids = [];
let bullets = [];

// boolean to define whether to show the main menu or the game
let isInGame = false;

// ammount of asteroids spawned in setup
let amountAsteroids = 4;

// delay until the player can fire again
let fireDelay = 20;
let fireCounter = 0;

// scoring variables
let playerScore = 0;
let highScore = 0;

function setup() {
  createCanvas(windowWidth, windowHeight); // define canvas (required for p5.js)

  // push all of the asteroids into their array
  for(let i = 0; i < amountAsteroids; i++){
    asteroids.push(new asteroid())
  }

  // fully define here because width / height is not defined before the creation of the canvas
  playerX = width/2;
  playerY = height/2;
}

function draw() {
  if(isInGame == true){
    background(0); // draw the black background
    player(); // update the player

    // update all of the asteroids
    for(let i = 0; i < asteroids.length; i++){
      asteroids[i].view(); // draw the itterated asteroid
      asteroids[i].update(); // update the position of the asteroid
    }

    // update all of the bullets
    for(let i = 0; i < bullets.length; i++){
      bullets[i].view(); // draw the itterated bullet
      bullets[i].update(); // update it's position
    }
      
    // formatting
    stroke(255);
    fill(0);
    textAlign(LEFT);

    // text describing the current and high scores
    text("Current Score: " + playerScore, 50, 50);
    text("High Score: " + highScore, 50, 150);

  } 
  else{
    // formatting
    background(100);
    textSize(50);
    textAlign(CENTER);
    fill(0);

    // text on the screen
    text("Press Space To Play", width/2, height/2);    
    text("High Score: " + highScore, width/2, height/2 - 100);
  }
}

function player(){
  // formatting
  fill(200);
  circle(playerX, playerY, 50); // draw the player

  // player controls
  if(keyIsDown(87)) playerY -= playerSpeed; // move up
  if(keyIsDown(83)) playerY += playerSpeed; // move down
  
  if(keyIsDown(65)) playerX -= playerSpeed; // move left
  if(keyIsDown(68)) playerX += playerSpeed; // move right

  // player collisions
  for(let i = 0; i < asteroids.length; i++){
    let d = dist(playerX, playerY, asteroids[i].x, asteroids[i].y); // get the distance from the player to the asteroid

    // if the distance between the player and the itterated asteroid is less than the sum of their radii, call the "die" function 
    if(d < (50+asteroids[i].radius)/2) die();
  }

  // fire bullets at the asteroids

  fireCounter ++; // itterate the timer that determines whether the player can fire another bullet or not
  if(fireCounter >= fireDelay && keyIsDown(32)){
    bullets.push(new bullet(playerX, playerY)); // create the new bullet
    fireCounter = 0; // ensure the player can not simply hold the shoot button and create infinite bullets
  }

  if(playerScore > highScore) highScore = playerScore; // update high score
}

function die(){
  // reset all asteroids
  for(let i = 0; i < asteroids.length; i++){
    asteroids[i].reset();
  }

  bullets = []; // clear bullets array

  // reset player position
  playerX = width/2;
  playerY = height/2;

  isInGame = false; // update bool to reset to main menu
}

// keyPressed is only called the frame a key is pressed, unlike keyIsDown(...) which is called every frame said key is down
// this means if you die holding space(to fire bullets), you still need to press space again in order to restart the game
function keyPressed(){
  // starting game / menu -> game
  if(keyCode === 32) isInGame = true;
}

class bullet{
  constructor(x, y){
    this.x = x; // starting X Position of the bullet
    this.y = y; // starting Y Position of the bullet

    // determines how long the bullet exists before it removes itself from the array
    this.lifeSpan = 75;

    // cur timer is constantly updated and is compared against this.lifespan to determine when it should "die"
    this.curTimer = 0;
    
    // vector used to update the position of the bullet every frame
    this.yMovement = -5;
  }
  update(){
    // update the life span timer
    this.curTimer ++;
    if(this.curTimer >= this.lifeSpan) bullets.shift(); // .shift() removes the last index of the arary or array[array.length] = undefined

    this.y += this.yMovement; // update the actual position of the bullet.

    // collisions
    for(let i = 0; i < asteroids.length; i++){
      let d = dist(this.x, this.y, asteroids[i].x, asteroids[i].y); // get the distance between the bullet and itterated asteroid
      
      if(d < (10 + asteroids[i].radius)/2) {
        asteroids[i].reset(); // asteroid destruction
        this.x = -100; // not necesarily "destroyed" but the bullet wont interact with anything else and will die b/c of lifespan naturally
        playerScore ++; // update the score
      }
    }
  }
  view(){
    // formatting
    fill(255);
    stroke(0);

    // draw the actual bullet
    circle(this.x, this.y, 10);
  }
}

class asteroid{
  constructor(){
    this.x = random(0, width); // starting X Position of the asteroid
    this.y = 0; // starting Y Position of the asteroid
    this.radius = 50; // starting Radius of the asteroid
    this.ySpeed = random(5, 20); // define a random movement vector that applies every frame
  }
  reset(){
    // reset the position
    this.x = random(0, width);
    this.y = -50;
    this.radius = random(30, 100);

    // reset the speed to a different value
    this.ySpeed = random(5, 20);
  }
  update(){
    this.y += this.ySpeed; // update the y position
    if(this.y > height + 10) this.reset(); // if it is off of the screen, reset it
  }
  view(){
    // formatting
    fill(100);
    stroke(0);

    // draw the asteroid
    circle(this.x, this.y, this.radius);
  }
}
