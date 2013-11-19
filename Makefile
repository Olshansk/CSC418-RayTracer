CC = g++
CFLAGS = -g -O2
LIBS = -lm

raytracer:  raytracer.o util.o light_source.o scene_object.o bmp_io.o render_style.o
	$(CC) $(CFLAGS) -o raytracer \
	raytracer.o util.o light_source.o scene_object.o bmp_io.o render_style.o $(LIBS)

clean:
	-rm -f core *.o
	-rm raytracer

scene-signature: raytracer
	./raytracer --scene-signature

ambient-diffuse: raytracer
	./raytracer --ambient-diffuse

phong: raytracer
	./raytracer --phong

bmp: scene-signature ambient-diffuse phong

png: bmp
	mogrify -format png *.bmp
