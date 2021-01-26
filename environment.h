const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

void free( SDL_Texture** texture );
//Make an image into an SDL_Texture
SDL_Texture* loadFromFile( std::string path, 
		SDL_Renderer* renderer, SDL_Texture* prevTexture );

bool init( SDL_Window** window, SDL_Renderer** renderer );
void close( SDL_Window** window, SDL_Renderer** renderer, SDL_Texture** texture=NULL );

bool init( SDL_Window** window, SDL_Renderer** renderer )
{
	if( SDL_Init( SDL_INIT_VIDEO ) != 0 ) {
		printf( "Failed to initialize video %s\n", SDL_GetError() );
		return false;
	} 

	*window = SDL_CreateWindow( 
			"Window", 
			SDL_WINDOWPOS_UNDEFINED,
			SDL_WINDOWPOS_UNDEFINED,
			SCREEN_WIDTH,
			SCREEN_HEIGHT,
			0
	);
	if( *window == NULL ) {
		printf( "Could not create window: %s\n", SDL_GetError() );
		return false;	
	} 

	*renderer = SDL_CreateRenderer( *window, -1, SDL_RENDERER_ACCELERATED );
	if( *renderer == NULL ) {
		printf( "Could not create renderer: %s\n" );
		return false;
	}
	SDL_SetRenderDrawColor( *renderer, 0xFF, 0xFF, 0xFF, 0xFF );

	int imgFlags = IMG_INIT_PNG;
	if( !( IMG_Init( imgFlags ) & imgFlags ) ) {
		printf( "SDL_image could not initialize! %s\n", IMG_GetError() );
		return false;
	}

	return true;
}

void close( SDL_Window** window, SDL_Renderer** renderer, SDL_Texture** texture )
{          
	free( texture );

	SDL_DestroyWindow( *window );
	*window = NULL;

	SDL_DestroyRenderer( *renderer );
	*renderer = NULL;
	
	IMG_Quit();
	SDL_Quit();
}

//Free a texture
void free( SDL_Texture** texture ) 
{
	if( *texture != NULL ) {
		SDL_DestroyTexture( *texture );
		*texture = NULL;
	}
}

//Make an image into an SDL_Texture
SDL_Texture* loadFromFile( std::string path, 
		SDL_Renderer* renderer, SDL_Texture* prevTexture )
{
	free( & prevTexture );
	
	SDL_Texture* newTexture = NULL;

	//Load image at specified path
	SDL_Surface* loadedSurface = IMG_Load( path.c_str() );
	if( loadedSurface == NULL ) {
		printf( "Unable to load image %s! SDL_image Error: %s\n", path.c_str(), IMG_GetError() );
		return NULL;
	} 

	//Color key image
	SDL_SetColorKey( loadedSurface, SDL_TRUE, SDL_MapRGB( loadedSurface->format, 0, 0xFF, 0xFF ) );

	//Create texture from surface pixels
	newTexture = SDL_CreateTextureFromSurface( renderer, loadedSurface );
	if( newTexture == NULL ) {
		printf( "Unable to create texture from %s! SDL Error: %s\n", path.c_str(), SDL_GetError() );
		return NULL;
	}

	//Get rid of old loaded surface
	SDL_FreeSurface( loadedSurface );

	return newTexture;
}

