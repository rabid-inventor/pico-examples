/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#define PI 3.14159265
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hub75.pio.h"
#include "pico/multicore.h"
#include "hardware/adc.h"

#include "mountains_128x64_rgb565.h"
#define PICO_DEFAULT_UART_TX_PIN 20
#define	PICO_DEFAULT_UART_RX_PIN 21

#define LED_R 16
#define LED_G 17
#define LED_B 18 
#define BUTTON_A    14
#define BUTTON_B    23

#define DATA_BASE_PIN 0
#define DATA_N_PINS 6
#define ROWSEL_BASE_PIN 6
#define ROWSEL_N_PINS 5
#define CLK_PIN 11
#define STROBE_PIN 12
#define OEN_PIN 13

#define WIDTH 64
#define HEIGHT 64

#define FLAG_VALUE 123
static uint32_t gc_row[HEIGHT][WIDTH]; // Global frame buffer 

static inline uint32_t gamma_correct_565_888(uint16_t pix) {
    uint32_t r_gamma = pix & 0xf800u;
    r_gamma *= r_gamma;
    uint32_t g_gamma = pix & 0x07e0u;
    g_gamma *= g_gamma;
    uint32_t b_gamma = pix & 0x001fu;
    b_gamma *= b_gamma;
    return (b_gamma >> 2 << 16) | (g_gamma >> 14 << 8) | (r_gamma >> 24 << 0);
}

/*
below claculation is taken from https://stackoverflow.com/questions/16124127/improvement-to-my-mandelbrot-set-code
*/
int w = WIDTH, h = HEIGHT;
double pr, pi;                   //real and imaginary part of the pixel p
double newRe, newIm, oldRe, oldIm;   //real and imaginary parts of new and old z
//double zoom = 1, moveX = -0.5, moveY = 0; //you can change these to zoom and change position
int maxIterations = 100;//after how much iterations the function should stop

uint32_t mandelbrot( int x ,int y, double zoom , double moveX, double moveY ){
    uint32_t ret;
    //calculate the initial real and imaginary part of z, based on the pixel location and zoom and position values
    pr = 1.5 * (x - w / 2) / (0.5 * zoom * w) + moveX;
    pi = (y - h / 2) / (0.5 * zoom * h) + moveY;
    newRe = newIm = oldRe = oldIm = 0; //these should start at 0,0
    //"i" will represent the number of iterations
    int i;
    //start the iteration process
    for(i = 0; i < maxIterations; i++)
    {
        //remember value of previous iteration
        oldRe = newRe;
        oldIm = newIm;
        //the actual iteration, the real and imaginary part are calculated
        newRe = oldRe * oldRe - oldIm * oldIm + pr;
        newIm = 2 * oldRe * oldIm + pi;
        //if the point is outside the circle with radius 2: stop
        if((newRe * newRe + newIm * newIm) > 4) break;
    }
    if(i=0){
        ret = 0x00000000;
    }
    else{
        ret = (1 << 16) | (1 << 8) | ((i *5) % 255); 
    }

    
    return ret;



}
void DisplayUpdateCore1(){
    PIO pio = pio0;
    uint sm_data = 0;
    uint sm_row = 1;

    uint data_prog_offs = pio_add_program(pio, &hub75_data_rgb888_program);
    uint row_prog_offs = pio_add_program(pio, &hub75_row_program);
    hub75_data_rgb888_program_init(pio, sm_data, data_prog_offs, DATA_BASE_PIN, CLK_PIN);
    hub75_row_program_init(pio, sm_row, row_prog_offs, ROWSEL_BASE_PIN, ROWSEL_N_PINS, STROBE_PIN);
    while (1) {
        //printf("Core1 loop");
            for (int rowsel = 0; rowsel < (1 << ROWSEL_N_PINS); ++rowsel) {
                /*
                for (int x = 0; x < WIDTH; ++x) {
                    gc_row[0][x] = mandelbrot(x, rowsel,1, -0.5,0);
                    gc_row[1][x] = mandelbrot(x, rowsel+16,1, -0.5,0);
                }
                */
                for (int bit = 0; bit < 8; ++bit) {
                    hub75_data_rgb888_set_shift(pio, sm_data, data_prog_offs, bit);
                    for (int x = 0; x < WIDTH; ++x) {
                        pio_sm_put_blocking(pio, sm_data,gc_row[x][rowsel] ); //gc_row[0][x]
                        pio_sm_put_blocking(pio, sm_data, gc_row[x][rowsel + (WIDTH/2)]); //gc_row[1][x]
                    }
                    // Dummy pixel per lane
                    pio_sm_put_blocking(pio, sm_data, 0);
                    pio_sm_put_blocking(pio, sm_data, 0);
                    // SM is finished when it stalls on empty TX FIFO
                    hub75_wait_tx_stall(pio, sm_data);
                    // Also check that previous OEn pulse is finished, else things can get out of sequence
                    hub75_wait_tx_stall(pio, sm_row);

                    // Latch row data, pulse output enable for new row.
                    pio_sm_put_blocking(pio, sm_row, rowsel | (100u * (1u << bit) << 5));
                }

            }
            /*
            sleep_ms(200);
            gpio_put(LED_R, 1);
            sleep_ms(200);
            gpio_put(LED_R, 0);
            */
        }
}   
uint32_t HSVtoRGB(float H, float S,float V){
    
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-fabs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    int R = (r+m)*255;
    int G = (g+m)*255;
    int B = (b+m)*255;

    return ((R << 16 ) | (G << 8) | B ); 
}

uint32_t checker(int x, int y, int step){
    x = x - (WIDTH/2);
    y = y - (HEIGHT/2);

    float angle = (step / 10.0);
    float s = sin(angle);
    float c = cos(angle);

    float xs = x * c - y * s;
    float ys = x * s + y * c;

    xs =  xs - sin(step / 200.0) * 40.0;
    ys =  ys - cos(step / 200.0) * 40.0;

    int scale = step % 20;
    scale /= 20;
    scale = (sin(step / 50.0) / 8.0) + 0.25;

    xs = xs * scale;
    ys = ys * scale;

    int xo = fabsf(xs) - (int)fabsf(xs);
    int yo = fabsf(ys) - (int)fabsf(ys);
    float l;
    
    if ((int)(floor(xs) + floor(ys)) > 1)  {
        l = 0;
    }
    else if( (xo > 0.5)  ){
        if((yo > 0.5)){
        l = 1;
        }
    }     
    else{ 
        l = 0.5;
        //l=0.1;
    }

    return HSVtoRGB((step % 360) , 1 * 100, l * 10);

    
}

const float voltageConversionFactor = (3.3f / (1 << 12));
const float currentConversionFactor = (3.3f / (1 << 12)/0.75);

float get_adc_voltage(int adcChannel){
    adc_select_input(adcChannel);
    uint16_t result = adc_read(); //grab raw adc value 
    float voltage;
    voltage = result * voltageConversionFactor;
    return voltage;

}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    adc_init();
    adc_gpio_init(29);
    adc_gpio_init(28);
    adc_gpio_init(27);
    adc_gpio_init(26);
    gpio_disable_pulls(29);
    gpio_disable_pulls(28);
    gpio_disable_pulls(27);
    gpio_disable_pulls(26);
    adc_select_input(3);
    gpio_init(LED_R);
    gpio_set_dir(LED_R, GPIO_OUT);
    gpio_put(LED_R, 1);
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_put(LED_G, 1);
    gpio_init(LED_B);
    gpio_set_dir(LED_B, GPIO_OUT);
    gpio_put(LED_B, 1);
    gpio_init(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_A,GPIO_IN);
    gpio_set_dir(BUTTON_B,GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_pull_up(BUTTON_B);

    //fill array
    printf("start");

    
    //printf("rbuffer loaded ");
    //printf("starting Core1");
    multicore_launch_core1(DisplayUpdateCore1);
    //printf("core1 started");
    uint8_t counter;
    uint16_t step;
    
    

    
    while(1){
        //printf("Core0 loop");



        
    for (int16_t x=0; x < HEIGHT; x++ ){
        for (int16_t y=0; y < WIDTH; y++ ){
            //gc_row[x][y] = mandelbrot(x, y,(double)step, -0.5,0.0);
            gc_row[x][y] = HSVtoRGB(step , 100, 40 );
            //gc_row[x][y] = 0xFFFFFFFF;
            //gc_row[x][y] = checker(x,y,step);
        


        }
        }
    step = step +5;
    if (gpio_get(BUTTON_A)){
            gpio_put(LED_B, 1);
        }
        else {
            gpio_put(LED_B,0);
        }

        if (gpio_get(BUTTON_B)){
            gpio_put(LED_R, 1);
        }
        else {
            gpio_put(LED_R,0);
        }
        
        counter++;
        if (counter = 150){
            printf("CH1: %f CH2: %f CH3: %f \n", get_adc_voltage(0),get_adc_voltage(1),get_adc_voltage(2));
            uint16_t result = adc_read();
            /*
            if (result > 0x030){  // this number allows it to trigger with just a few leds 
                gpio_put(LED_R, 0);
            }
            else {
                gpio_put(LED_R, 1);
            }
            */
            printf("Raw value: 0x%03x, voltage: %f A\n", result, (result * currentConversionFactor));
        }
    /*
    sleep_ms(200);
    gpio_put(LED_G, 1);
    sleep_ms(200);
    gpio_put(LED_G, 0);
    */
    /*
   sleep_ms(200);
    gpio_put(LED_R, 0);
    sleep_ms(200);
    gpio_put(LED_R, 1);
    sleep_ms(200);
    gpio_put(LED_G, 0);
    sleep_ms(200);
    gpio_put(LED_G, 1);
    sleep_ms(200);
    gpio_put(LED_B, 0);
    sleep_ms(200);
    gpio_put(LED_B, 1);
    */
    }
    

    

}
