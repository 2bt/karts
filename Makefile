CF = -std=c++17 -Wall -O2 $(shell pkg-config --cflags bullet)
LF = -Wall -lSDL2 -lSDL2_image -lGLEW -lGL -lBulletDynamics -lBulletCollision -lLinearMath

CXX = g++
SRC = $(wildcard src/*.cpp)
OBJ = $(SRC:src/%.cpp=obj/%.o)
TRG = karts

all: $(TRG)


obj/%.o: src/%.cpp Makefile
	@mkdir -p obj/
	$(CXX) $(CF) $< -c -o $@


$(TRG): $(OBJ) Makefile
	$(CXX) $(OBJ) -o $@ $(LF)


clean:
	rm -rf obj/ $(TRG)


# compile it for the browser via emscripten
# hacky but works
browser:
	em++ -s WASM=1 -s USE_WEBGL2=1 -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s SDL2_IMAGE_FORMATS='["png"]'\
	     --preload-file media -std=c++1z -O2 -I./include $(SRC) -o docs/index.html --shell-file shell.html
