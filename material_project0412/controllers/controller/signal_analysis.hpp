#include <math.h>
#include "kiss_fft/kiss_fft.h" // FFT library
// #define SIGNAL_LENGTH 1024 // length of the signal to be analyzed
/**
* @brief KISS FFT USAGE EXAMPLE
* @param signal_data pointer to the array containing the signal to be analyzed
*/

#define THRESHOLD_FFT 0.05
#define SIGNAL_LENGTH 150

int find_peaks(double* signal_data){

    /* fft preparation */
    kiss_fft_cfg cfg = kiss_fft_alloc(SIGNAL_LENGTH, 0, NULL, NULL );
    /* fft variables */
    kiss_fft_cpx cx_in[SIGNAL_LENGTH], // input signal (time domain)
                 cx_out[SIGNAL_LENGTH]; // output signal (frequency domain)

    // prepare the input (add signal in the 'in' structure of the kiss_fft)
    for (int n=0; n<SIGNAL_LENGTH; n++) {
        cx_in[n].r = signal_data[n]; // the real part of the signal is the data
        cx_in[n].i = 0.; // set the imaginary part to zero
    }
    // run the fft (the fourier transform is stored in the 'out' structure of the kiss_fft)
    kiss_fft( cfg , cx_in , cx_out );
    // compute the magnitude of the complex numbers
    double mag[SIGNAL_LENGTH/2];
    for (int n=1; n<SIGNAL_LENGTH/2; n++) {
        mag[n] = sqrt(cx_out[n].r*cx_out[n].r + cx_out[n].i*cx_out[n].i) / SIGNAL_LENGTH;
    }
    // Find peaks
    int num_peaks = 0;
    for(int i = 1; i<SIGNAL_LENGTH/2 -1; i++){
        if (mag[i]>THRESHOLD_FFT) num_peaks++;
        //printf("%f\n",mag[i]);
    }
    // free fft memory once done with it
    free(cfg);

    return num_peaks;
}
enum conditions { INIT, BAD, GOOD };  

extern double global_max;
extern double local_max ;
extern double positionx;
extern double positiony;
extern double old_local_max;
extern int light_condition;
extern int search_max;
extern int time_step_ctr; 
extern int light_ctr ;
extern bool light_found;
extern bool ready ;
extern int good_ctr;
extern int bad_ctr;

void light_analysis(double light, double* moving_window, Pose * pose){
    //printf("%f\n",light); 

    for(int i=0;i<SIGNAL_LENGTH;i++){
      moving_window[i]=moving_window[i+1];
    }
    moving_window[SIGNAL_LENGTH-1]=light;

    int n_peaks = find_peaks(moving_window);

    switch(light_condition){
      case INIT:
        //printf("%d\n",time_step_ctr);
        if(time_step_ctr++>SIGNAL_LENGTH)light_condition = GOOD;
        break;
      
      case GOOD:
        good_ctr=0;
        //printf("gooood\n");
        if(n_peaks>1){
          if(bad_ctr++>20) light_condition=BAD;
        }
        if(light>2.2){
          if(moving_window[SIGNAL_LENGTH-1]>global_max && !light_found){
            //printf("test\n");
            global_max=moving_window[SIGNAL_LENGTH-1];
            positionx = pose->x;
            positiony = pose->y;
            //printf("global: %f", global_max);
          }
          else if(!light_found){
            //printf("smaller\n");
            local_max = moving_window[SIGNAL_LENGTH-1];
            for (int i = SIGNAL_LENGTH - 2; i >= SIGNAL_LENGTH - 50 && i >= 0; i--) {
              if (moving_window[i] > local_max) {
                local_max = moving_window[i];
              }
            }
            //printf("new: %f\n", local_max);
            if(global_max>local_max){
              //printf("%f, %f\n",old_local_max, local_max);
              printf("Detected light n°%d, status: good, location: (%.3lf,%.3lf)m \n", light_ctr++,positionx,positiony);
              //printf("%f\n",global_max);
              light_found=true;
              local_max = 2.1;
              global_max= 2.1;
              positionx = 0;
              positiony = 0;
            }
          }
        }
        if(light<1.8) light_found = false;
        
        break;

      case BAD:
      bad_ctr = 0;
      //printf("bad\n");
      if(n_peaks<2){
        if(good_ctr++>50) light_condition=GOOD;
      }
      else good_ctr=0;
      local_max = moving_window[SIGNAL_LENGTH-1];
            for (int i = SIGNAL_LENGTH - 2; i >= SIGNAL_LENGTH - 50 && i >= 0; i--) {
              if (moving_window[i] > local_max) {
                local_max = moving_window[i];
              }
            }
      if(light>2.2){
          if(moving_window[SIGNAL_LENGTH-1]>global_max && !light_found){
            //printf("test\n");
            global_max=moving_window[SIGNAL_LENGTH-1];
            positionx = pose->x;
            positiony = pose->y;
            //printf("global: %f", global_max);
          }
          else if(!light_found){
            //printf("smaller\n");
            //printf("new: %f\n", local_max);
            if(global_max>local_max){
              //printf("%f, %f\n",old_local_max, local_max);
              printf("Detected light n°%d, status: bad, location: (%.3lf,%.3lf)m \n", light_ctr++,positionx,positiony);
              //printf("%f\n",global_max);
              light_found=true;
              local_max = 2.1;
              global_max= 2.1;
              positionx = 0;
              positiony = 0;
            }
          }
        }
        if(local_max<1.8) light_found = false;
        break;
    }
    //printf("%d\n",n);
}
