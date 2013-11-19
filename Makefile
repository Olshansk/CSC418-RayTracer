CC = g++
CFLAGS = -g -O2
LIBS = -lm

raytracer:  raytracer.o util.o light_source.o scene_object.o bmp_io.o render_style.o
	$(CC) $(CFLAGS) -o raytracer \
	raytracer.o util.o light_source.o scene_object.o bmp_io.o render_style.o $(LIBS)

clean:
	-rm -f core *.o
	-rm raytracer

run: raytracer
	./raytracer --phong

png: run
	mogrify -format png *.bmp
