build:
	clang++ -std=c++17 -Wall ./src/*.cpp ./src/Physics/*.cpp -I/opt/homebrew/include/ \
-L/opt/homebrew/lib -lm -lSDL2 -lSDL2_image -lSDL2_gfx -o app

run:
	./app

clean:
	rm app
