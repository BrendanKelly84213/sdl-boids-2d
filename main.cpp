#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "environment.h"
#include "timer.h"

const int BOID_WIDTH  = 60 ;
const int BOID_HEIGHT = 30;
const int STEPS = 40;
const int BOID_SPEED = 7;
const double MAX_AVOID = 0.12;
const double MAX_ALN   = 0.00047;
const double MAX_CSN   = 0.000006;
const double BOID_SIGHT_RADIUS = 120;
const double DEG_TO_RAD = 0.01745329;
const double OFFSET = 180;
constexpr double pi = std::atan(1)*4; 

const int SCREEN_FPS = 60;
const int SCREEN_TICKS_PER_FRAME = 1000 / SCREEN_FPS;


const int DEFAULT_NUM_BOIDS = 600;

int NUM_BOIDS;

struct Vec2
{
	double x,y;

	Vec2() : x(0), y(0) {}

	bool operator!=( Vec2 a) {
		if( !( a.x == this->x && a.y == this->y ) ) {
			return true;
		}
		return false;
	}
};

typedef struct 
{
	Vec2 vel;
	Vec2 pos; 
	Vec2 accel; 
	SDL_Rect box;
}Boid;

int wrap( double in, int param ) 
{
	return in <= 0 ? param : static_cast<int>(round((in))) % param;
}

double wrap( double in, double param ) 
{
	if( in <= 0 ) {
		return param;
	}
	if( in >= param ) {
		return 0;
	}
	return in;
}

Vec2 addMod( Vec2 a, Vec2 b ) 
{
	Vec2 added;
	added.x = wrap( a.x + b.x, SCREEN_WIDTH );
	added.y = wrap( a.y + b.y, SCREEN_HEIGHT );
	return added;
}

Vec2 add( Vec2 a, Vec2 b ) 
{
	Vec2 added;
	added.x = a.x + b.x;
	added.y = a.y + b.y;
	return added;
}

Vec2 sub( Vec2 a, Vec2 b ) 
{
	Vec2 subbed;
	subbed.x = a.x - b.x;
	subbed.y = a.y - b.y;
	return subbed;
}

Vec2 scale( Vec2 a, double b ) 
{
	Vec2 cpy;
	cpy.x = a.x * b;
	cpy.y = a.y * b;
	return cpy;
}

//Computes the direction angle of a vector
double vecDir( Vec2 v ) 
{
	return (std::atan( v.y / v.x ) / DEG_TO_RAD) - OFFSET;
}

//Computes the magnitude of a vector
double vecMag( Vec2 v )
{
	return std::sqrt( v.x*v.x + v.y*v.y ); 
}

Vec2 norm( Vec2 v ) 
{
	return scale( v, (1/vecMag(v)));
}

Vec2 newMag( Vec2 v, double n )
{
	return scale( norm( v ), n );
}

Vec2 limit( Vec2 v, double l ) 
{
	double mSq = vecMag(v)*vecMag(v);
	if( mSq > l*l ) {
		v = newMag( v, mSq );
		v = scale( v, l );
	}
	return v;
}

Vec2 minLimit( Vec2 v, double l ) 
{
	double mSq = vecMag(v)*vecMag(v);
	if( mSq < l*l ) {
		v = newMag( v, mSq );
		v = scale( v, l );
	}
	return v;
}

Boid randBoid() 
{
	Boid b;
	b.pos.x = 20+rand() % (SCREEN_WIDTH - 20);
	b.pos.y = 20+rand() % (SCREEN_HEIGHT - 20);
	b.vel.x = -3*BOID_SPEED + (rand() % 6*BOID_SPEED);
	b.vel.y = -3*BOID_SPEED + (rand() % 6*BOID_SPEED); 
	b.vel = newMag( b.vel, BOID_SPEED );
	b.accel.x = 0;
	b.accel.y = 0;
	b.box = { b.pos.x - BOID_WIDTH/2, b.pos.y - BOID_HEIGHT/2, 
		BOID_WIDTH, BOID_HEIGHT };
	return b;
}

Boid moveBoid( Boid boid )
{
	Boid b;
	b.vel = boid.vel;
	b.pos = addMod( b.pos, boid.vel );
	b.box = { b.pos.x - BOID_WIDTH/2, b.pos.y - BOID_HEIGHT/2, 
		BOID_WIDTH, BOID_HEIGHT };
	return b;
}

double dist( double x0, double x1, double y0, double y1 ) 
{
	return std::sqrt( (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0) );
}

/*

Boid logic

separation: steer to avoid crowding local flockmates (a collision?)
alignment: steer towards the average heading of local flockmates
cohesion: steer to move towards the average position (center of mass) of local flockmates

*/

enum Flag { ALIGN, COHESE, AVOID };

// Direction of boids acceleration
Vec2 steer( Boid current, Boid boids[DEFAULT_NUM_BOIDS], Flag f ) 
{
	Vec2 avg, steer;
	double x=current.pos.x;
	double y=current.pos.y;

	int total = 0;
	for( int i=0; i < NUM_BOIDS; ++i ) {
		double d = dist( x, boids[i].pos.x, y, boids[i].pos.y );

		auto t = [&]( Boid b ) { 
			switch( f ) {
				case ALIGN: return b.vel; break;
				case COHESE: return b.pos; break;
				case AVOID: 
					Vec2 diff = sub( current.pos, b.pos );
					return scale( diff, (1/(d*d)) );
			}
		};

		if( current.vel != boids[i].vel 
		 && current.pos != boids[i].pos 
		 && d < BOID_SIGHT_RADIUS ) {
		 	avg = add( avg, t( boids[i] ) );
			total++;
		 }
	}

	if( total > 0 ) {
		avg.x /= total;
		avg.y /= total;

		if( std::isnan( vecMag( avg ) ) ) {
			// Outputs -nan
			/* std::cout << vecMag( avg ) << '\n'; */ 
			return Vec2();
		}

		if( f == COHESE ) {
			avg = sub( avg, current.pos );
		}

		steer = sub( avg, current.vel );
	}

	auto scaler = [&]() {
		switch( f ) {
			case ALIGN: return MAX_ALN; break;
			case COHESE: return MAX_CSN; break;
			case AVOID: return MAX_AVOID; break;
		}
	};

	return limit( steer, scaler() );
}

int main( int argc, char *argv[] ) 
{
	SDL_Window*   window   = NULL;  
	SDL_Renderer* renderer = NULL;

  	//The frames per second timer
       	LTimer fpsTimer;

        //The frames per second cap timer
        LTimer capTimer;

        //Start counting frames per second
        int countedFrames = 0;
        fpsTimer.start();

	// Set number of boids
	if( !argv[1] ) {
		NUM_BOIDS = DEFAULT_NUM_BOIDS;
		std::cout << "No argument detected" << '\n';
	} else  {
		NUM_BOIDS = atoi( argv[1] ); 
		std::cout << "agrument" << NUM_BOIDS << '\n';
	}

	// Init window
	if( !init( & window, & renderer ) ) {
		return 1;	
	}
	
	SDL_Event e;
	bool running = true;

	SDL_Texture* texture = NULL; 
	texture=loadFromFile( "images/fish3.png", renderer, texture );
	int counter=0;

	Boid boids[NUM_BOIDS];

	// Update window size 
	// Have to call SDL_PollEvent for some reason
	SDL_PollEvent( &e );
	SDL_GetWindowSize( window, &SCREEN_WIDTH, &SCREEN_HEIGHT );

	for( int i=0; i < NUM_BOIDS; ++i ) {
		boids[i] = randBoid();
	}

	while( running ) {
		if( SDL_PollEvent( &e ) ) {
			if( e.type == SDL_KEYDOWN ) {
				switch( e.key.keysym.sym) {
					case SDLK_q:
						running = false;
					break;
				}
			} 
			if( e.type == SDL_QUIT ) {
				running = false;
			}
		}

                //Start cap timer
                capTimer.start();

                //Calculate and correct fps
                float avgFPS = countedFrames / ( fpsTimer.getTicks() / 1000.f );
                if( avgFPS > 2000000 ) {
                    avgFPS = 0;
                }

		// Clear screen
		SDL_SetRenderDrawColor( renderer, 0, 0, 0, SDL_ALPHA_OPAQUE );
		SDL_RenderClear( renderer );

		// Update window size
		SDL_GetWindowSize( window, &SCREEN_WIDTH, &SCREEN_HEIGHT);

		for( int i=0; i < NUM_BOIDS; ++i ) {
			double angle = vecDir( boids[i].vel );
			boids[i].pos = addMod( boids[i].pos, boids[i].vel );
			boids[i].vel = add( boids[i].vel, boids[i].accel );

			Vec2 steering = add( steer( boids[i], boids, ALIGN ), 
                                        add( steer( boids[i], boids, AVOID ), 
					     steer( boids[i], boids, COHESE ) )) ;

			boids[i].accel = steering; 

			if(vecMag(boids[i].vel) < BOID_SPEED) {
				boids[i].vel = newMag(boids[i].vel, BOID_SPEED);
			}

			boids[i].box = { boids[i].pos.x - BOID_WIDTH/2, boids[i].pos.y - BOID_HEIGHT/2, 
				BOID_WIDTH, BOID_HEIGHT };

			SDL_SetRenderDrawColor( renderer, 0, 0, 0, SDL_ALPHA_OPAQUE );
			SDL_RenderCopyEx( renderer, texture, NULL, 
					&boids[i].box, angle, NULL, SDL_FLIP_VERTICAL ); 
		}

		//Update screen
		SDL_RenderPresent( renderer );
		//SDL_Delay(15);

		// If frame finished early
                int frameTicks = capTimer.getTicks();
                if( frameTicks < SCREEN_TICKS_PER_FRAME ) {
                    //Wait remaining time
                    SDL_Delay( SCREEN_TICKS_PER_FRAME - frameTicks );
                }
	}

	//Free and close SDL
	close( & window, & renderer, & texture );

	return 0;
}
