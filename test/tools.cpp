#include "tools.h"

cml::vector3d intersectLineTriangle(cml::vector3d X, cml::vector3d dir,
                                    cml::vector3d V0, cml::vector3d V1,
                                    cml::vector3d V2)
{
    double tmp = cml::dot( cml::cross(dir,V2-V0), (V1-V0) );
    tmp = 1.0/tmp;

    double t = tmp * cml::dot( cml::cross(X-V0,V1-V0), (V2-V0) );
    double u = tmp * cml::dot( cml::cross(dir,V2-V0), (X-V0) );
    double v = tmp * cml::dot( cml::cross(X-V0, V1-V0), dir );

    return X+t*dir;
}


/*
 * Copyright 2002-2008 Guillaume Cottenceau.
 *
 * This software may be freely redistributed under the terms
 * of the X11 license.
 *
 */
// taken from http://zarb.org/~gc/html/libpng.html

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define PNG_DEBUG 3
#include <png.h>

void abort_(const char * s, ...)
{
    va_list args;
    va_start(args, s);
    vfprintf(stderr, s, args);
    fprintf(stderr, "\n");
    va_end(args);
    abort();
}


IplImage* read_png_file(char* file_name)
{
    png_structp png_ptr;
    png_infop info_ptr;
    char header[8];	// 8 is the maximum size that can be checked

    // open file and test for it being a png
    FILE *fp = fopen(file_name, "rb");
    if (!fp)
        abort_("[read_png_file] File %s could not be opened for reading", file_name);
    fread(header, 1, 8, fp);
    if (png_sig_cmp((png_byte*)header, 0, 8))
        abort_("[read_png_file] File %s is not recognized as a PNG file", file_name);


    // initialize stuff
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr)
        abort_("[read_png_file] png_create_read_struct failed");

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
        abort_("[read_png_file] png_create_info_struct failed");

    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[read_png_file] Error during init_io");

    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);

    png_read_info(png_ptr, info_ptr);

    int width = info_ptr->width;
    int height = info_ptr->height;
    int color_type = info_ptr->color_type;
    int bit_depth = info_ptr->bit_depth;

    int number_of_passes = png_set_interlace_handling(png_ptr);
    png_read_update_info(png_ptr, info_ptr);


    // read file
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[read_png_file] Error during read_image");

    //row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
    //Create storage space
    IplImage* img = cvCreateImage(cvSize(width,height), bit_depth, 1);

    png_byte* row_pointers[height];
    for (int y=0; y<height; y++)
        row_pointers[y] = (png_byte*) img->imageData + y*img->widthStep;

    png_read_image(png_ptr, row_pointers);

    fclose(fp);

    return img;
}


void write_png_file(char* file_name, IplImage* img)
{
    png_structp png_ptr;
    png_infop info_ptr;

    /* create file */
    FILE *fp = fopen(file_name, "wb");
    if (!fp)
        abort_("[write_png_file] File %s could not be opened for writing", file_name);


    /* initialize stuff */
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

    if (!png_ptr)
        abort_("[write_png_file] png_create_write_struct failed");

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
        abort_("[write_png_file] png_create_info_struct failed");

    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during init_io");

    png_init_io(png_ptr, fp);


    /* write header */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during writing header");

    png_set_IHDR(png_ptr, info_ptr, img->width, img->height,
                 img->depth, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);


    /* write bytes */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during writing bytes");

    png_byte* row_pointers[img->height];
    for(int i=0; i<img->height; i++)
        row_pointers[i] = (png_byte*)img->imageData + i*img->widthStep;
    png_write_image(png_ptr, row_pointers);


    /* end write */
    if (setjmp(png_jmpbuf(png_ptr)))
        abort_("[write_png_file] Error during end of write");

    png_write_end(png_ptr, NULL);

    fclose(fp);
}


/*void process_file(void)
{
    if (info_ptr->color_type != PNG_COLOR_TYPE_RGBA)
        abort_("[process_file] color_type of input file must be PNG_COLOR_TYPE_RGBA (is %d)",
               info_ptr->color_type);

    for (y=0; y<height; y++) {
        png_byte* row = row_pointers[y];
        for (x=0; x<width; x++) {
            png_byte* ptr = &(row[x*4]);
            printf("Pixel at position [ %d - %d ] has the following RGBA values: %d - %d - %d - %d\n",
                   x, y, ptr[0], ptr[1], ptr[2], ptr[3]);

            // set red value to 0 and green value to the blue one
            ptr[0] = 0;
            ptr[1] = ptr[2];
        }
    }

}*/


