#include <stdbool.h>
#include <stdint.h>

#include "innerbloom_instrumental.h"
#include "innerbloom_vocals.h"
#include "ten_instrumental.h"
#include "ten_vocals.h"
#include "dj_background.h"

/*~~~~~
Colors
~~~~~~*/
#define BLACK 0x0000
#define WHITE 0xFFFF

#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define YELLOW 0xFFE0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define ORANGE 0xFD20
#define DARK_GRAY 0x18C3
#define DARKER_GRAY 0x1082
#define GRAY 0x8410

/*~~~~~~~~
Background
~~~~~~~~~~*/
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define DJ_BACKGROUND_WIDTH 320
#define DJ_BACKGROUND_HEIGHT 176

/*~~~~~~~~~~~~~~~~~~~~~
Hardware I/O addresses
~~~~~~~~~~~~~~~~~~~~~~~*/
#define LEDR_BASE 0xFF200000
#define SW_BASE 0xFF200040
#define KEY_BASE 0xFF200050
#define AUDIO_BASE 0xFF203040
#define TIMER_BASE 0xFF202000
#define PIXEL_CTRL_BASE 0xFF203020

/*~~~~~~~~~~~~~~~~~~~~~
Audio / effect settings
~~~~~~~~~~~~~~~~~~~~~~~*/
#define SAMPLE_RATE 8000
#define FADE_SECONDS 6
#define FADE_SAMPLES (SAMPLE_RATE * FADE_SECONDS)
#define ECHO_DELAY 2400

/*~~~~~~~~~~~
VGA settings
~~~~~~~~~~~~~~*/
#define LEFT_CX 88
#define LEFT_CY 116
#define RIGHT_CX 232
#define RIGHT_CY 116

/*~~~~~~~~~~~~~~~~~~~~~
Audio hardware struct
~~~~~~~~~~~~~~~~~~~~~~~*/
struct audio_t {
    volatile unsigned int control;
    volatile unsigned char rarc;
    volatile unsigned char ralc;
    volatile unsigned char wsrc;
    volatile unsigned char wslc;
    volatile unsigned int ldata;
    volatile unsigned int rdata;
};

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Struct that characterizes the state of the DJ board
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
typedef struct {
    int song1VocalsOn; /* 1 if active, 0 if not */
    int song1InstrOn; /* 1 if active, 0 if not */
    int song2VocalsOn; /* 1 if active, 0 if not */
    int song2InstrOn; /* 1 if active, 0 if not */
    int muteOn; /* 1 if active, 0 if not */
    int echoOn; /* 1 if active, 0 if not */
    int activeSong; /* 0 if song A is active, 1 if song B is active */
    int blockFade; /* 1 while fade is active / new fade blocked */
    int fadeSongAtoB; /* 1 if fading from A to B */
    int fadeSongBtoA; /* 1 if fading from B to A */
    int fadeCount; /* fade progress */

    int prevSW; /* previous switch value */
    int iA; /* index of song A */
    int iB; /* index of song B */
    int songLenA; /* length of song A */
    int songLenB; /* length of song B */
    int iEcho; /* index of echo buffer */
    int bpmNumerator; /*controla playback speed*/
    int bpmDenominator; /*controls playback speed*/
    int bpmCounterA; /*when to increment iA*/
    int bpmCounterB; /*when to increment iB*/
} audioState;

/*~~~~~~~~~~~~~~~
Declare Functions
~~~~~~~~~~~~~~~~~*/

//Audio Functions
void song1Vocals(volatile audioState *audio, int newState);
void song2Vocals(volatile audioState *audio, int newState);
void song1Instr(volatile audioState *audio, int newState);
void song2Instr(volatile audioState *audio, int newState);
void mute(volatile audioState *audio, int newState);
void echo(volatile audioState *audio, int newState);
void resetSong1(volatile audioState *audio);
void resetSong2(volatile audioState *audio);
int shortestSong(int A, int B);
bool write_sample(struct audio_t *audio_ptr, int sample);
int mixSong1(volatile audioState *audio);
int mixSong2(volatile audioState *audio);
void updateSong1(volatile audioState *audio);
void updateSong2(volatile audioState *audio);
int clip16(int x);
void start_fade(volatile audioState *audio);
int fadeSample(volatile audioState *audio);
void updateFade(volatile audioState *audio);
int apply_echo(volatile audioState *audio, int sample, int ring_buffer[]);
void raiseBPM(volatile audioState *audio);
void lowerBPM(volatile audioState *audio);

//VGA functions
void draw_arrow_left(void);
void draw_arrow_right(void);
void clear_arrow(void);
void fill_button1(void);
void fill_button2(void);
void fill_button3(void);
void fill_button4(void);
void fill_button5(void);
void fill_button6(void);
void plot_pixel(int x, int y, uint16_t color);
void clear_screen(uint16_t color);
void draw_image_centered(void);
void fill_rect(int x, int y, int width, int height, short int color);
void fill_circle(int cx, int cy, int r, short int color);
void wait_for_vsync(volatile int *pixel_ctrl_ptr);
void select_active_song_from_switches(volatile audioState *audio);
void request_vga_frame(void);
void render_vga_frame(volatile int *pixel_ctrl_ptr);
void init_vga_scene(volatile int *pixel_ctrl_ptr);
void clear_switch_boxes(void);
void draw_switch_boxes(void);
void draw_fade_arrow(int fade_arrow_target);

volatile audioState audio;
int ring_buffer[ECHO_DELAY] = {0};

short int Buffer1[240][512];
short int Buffer2[240][512];

volatile int pixel_buffer_start;

static volatile int fade_arrow_target = 0; /* 0 = left   1 = right */
static volatile int vga_redraw_request = 1;
static int vga_swap_pending = 0;
static volatile int vga_switch_mask = 0;

/* ISR prototypes */
static void handler(void) __attribute__((interrupt("machine")));
void set_timer(void);
void set_KEY(void);
void timerISR(void);
void KEY_ISR(void);

int main(void) {
    struct audio_t *audio_ptr = (struct audio_t *)AUDIO_BASE;
    volatile int *SW_ptr = (int *)SW_BASE;
    volatile int *pixel_ctrl_ptr = (int *)PIXEL_CTRL_BASE;

    int mstatus_value, mtvec_value, mie_value;
    int sample;

    /* initialize audio state */
    audio.song1VocalsOn = 0;
    audio.song1InstrOn = 0;
    audio.song2VocalsOn = 0;
    audio.song2InstrOn = 0;

    audio.muteOn = 0;
    audio.echoOn = 0;

    audio.activeSong = 0; /* 0 = song A active, 1 = song B active */
    audio.blockFade = 0;
    audio.fadeSongAtoB = 0;
    audio.fadeSongBtoA = 0;
    audio.fadeCount = 0;

    audio.prevSW = (*SW_ptr) & 0x7F; /* or 0x3F / 0x1F depending on switches used */
    audio.iA = 0;
    audio.iB = 0;
    audio.iEcho = 0;
    audio.bpmNumerator = 100;
    audio.bpmDenominator = 100;
    audio.bpmCounterA = 0;
    audio.bpmCounterB = 0;

    audio.songLenA = shortestSong(vocals_lenA, instrumental_lenA);
    audio.songLenB = shortestSong(vocals_lenB, instrumental_lenB);

    /* initialize VGA */
    int sw_now;

    sw_now = (*SW_ptr) & 0x7F;

    audio.prevSW = sw_now;

    /* apply current switch positions to start in correct state */
    song1Vocals(&audio, (sw_now >> 0) & 1);
    song1Instr(&audio, (sw_now >> 1) & 1);
    song2Vocals(&audio, (sw_now >> 2) & 1);
    song2Instr(&audio, (sw_now >> 3) & 1);
    mute(&audio, (sw_now >> 4) & 1);
    echo(&audio, (sw_now >> 6) & 1);

    select_active_song_from_switches(&audio);
    vga_switch_mask = sw_now & 0x7F;

    /* clear audio write FIFO */
    audio_ptr->control = 0x8;
    audio_ptr->control = 0x0;

    init_vga_scene(pixel_ctrl_ptr);

    /* set up timer + key interrupts */
    set_timer();
    set_KEY();

    mstatus_value = 0x8;
    asm volatile("csrc mstatus, %0" ::"r"(mstatus_value));

    mtvec_value = (int)&handler;
    asm volatile("csrw mtvec, %0" ::"r"(mtvec_value));

    asm volatile("csrr %0, mie" : "=r"(mie_value));
    asm volatile("csrc mie, %0" ::"r"(mie_value));

    mie_value = 0x50000; /* timer + key */
    asm volatile("csrs mie, %0" ::"r"(mie_value));

    asm volatile("csrs mstatus, %0" ::"r"(mstatus_value));

    while (1) {
        sample = fadeSample(&audio); //determines current sample

        if (audio.muteOn) { //if mute flag is on then mute
            sample = 0;
        }

        sample = apply_echo(&audio, sample, ring_buffer); //if echo is on apply delayed samples

        if (write_sample(audio_ptr, sample)) { //if the fifo has space advance song
            updateFade(&audio);
        }

        if (audio.blockFade) {
            request_vga_frame();
        }

        if (vga_swap_pending) {
            if ((*(pixel_ctrl_ptr + 3) & 0x1) == 0) {
                pixel_buffer_start = *(pixel_ctrl_ptr + 1);
                vga_swap_pending = 0;
            }
        }

        if (vga_redraw_request && !vga_swap_pending) {
            render_vga_frame(pixel_ctrl_ptr);
        }
    }

    return 0;
}

void handler(void) {
    int mcause_value;

    asm volatile("csrr %0, mcause" : "=r"(mcause_value));

    if (mcause_value == 0x80000010) {
        timerISR();
    } else if (mcause_value == 0x80000012) {
        KEY_ISR();
    }
}

void timerISR(void) {
    volatile int *timer_ptr = (int *)TIMER_BASE;
    volatile int *SW_ptr = (int *)SW_BASE;

    int sw_now;
    int changed;

    *timer_ptr = 0; /* clear timer interrupt */

    sw_now = (*SW_ptr) & 0x7F; /* use switches 0 to 6 */
    changed = sw_now ^ audio.prevSW;

    if (changed & 0x01) {
        song1Vocals(&audio, (sw_now >> 0) & 1);
    }

    if (changed & 0x02) {
        song1Instr(&audio, (sw_now >> 1) & 1);
    }

    if (changed & 0x04) {
        song2Vocals(&audio, (sw_now >> 2) & 1);
    }

    if (changed & 0x08) {
        song2Instr(&audio, (sw_now >> 3) & 1);
    }

    if (changed & 0x10) {
        mute(&audio, (sw_now >> 4) & 1);
    }

    if (changed & 0x40) {
        echo(&audio, (sw_now >> 6) & 1);
    }

    /* fade on SW[5] rising edge only */
    if (changed & 0x20) {
        if ((sw_now & 0x20) != 0) {
            start_fade(&audio);
        }
    }

    select_active_song_from_switches(&audio);
    audio.prevSW = sw_now;
    vga_switch_mask = sw_now & 0x7F;
    request_vga_frame();
}

void KEY_ISR(void) {
    volatile int *KEY_ptr = (int *)KEY_BASE;
    int pressed;

    pressed = *(KEY_ptr + 3); /* read edge capture */
    *(KEY_ptr + 3) = pressed; /* clear edge capture */

    if (pressed & 0x1) {
        resetSong1(&audio);
    }

    if (pressed & 0x2) {
        resetSong2(&audio);
    }

    if (pressed & 0x4) {
        raiseBPM(&audio);
    }

    if (pressed & 0x8) {
        lowerBPM(&audio);
    }
    request_vga_frame();
}

void set_timer(void) {
    volatile int *timer_ptr = (int *)TIMER_BASE;
    int load_val = 1000000;

    *(timer_ptr + 2) = (load_val & 0xFFFF);
    *(timer_ptr + 3) = (load_val >> 16) & 0xFFFF;
    *(timer_ptr + 1) = 0x7;
}

void set_KEY(void) {
    volatile int *KEY_ptr = (int *)KEY_BASE;

    *(KEY_ptr + 3) = 0xF;
    *(KEY_ptr + 2) = 0xF;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The following functions are set to update the values of the struct
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//the first few functions are meant to set off flags which deterine what will be sent out as the sample
void song1Vocals(volatile audioState *audio, int newState) { //sets flag to newstate
    audio->song1VocalsOn = newState;
}
void song2Vocals(volatile audioState *audio, int newState) { //sets flag to newstate
    audio->song2VocalsOn = newState;
}
void song1Instr(volatile audioState *audio, int newState) { //sets flag to newstate
    audio->song1InstrOn = newState;
}
void song2Instr(volatile audioState *audio, int newState) { //sets flag to newstate
    audio->song2InstrOn = newState;
}
void mute(volatile audioState *audio, int newState) { //sets flag to newstate
    audio->muteOn = newState;
}
void echo(volatile audioState *audio, int newState) { //sets flag to newstate
    audio->echoOn = newState;
}
void resetSong1(volatile audioState *audio) { //set inctement back to begining
    audio->iA = 0;
}
void resetSong2(volatile audioState *audio) { //set increment back to beginning
    audio->iB = 0;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~
Avoid songs being out of sync
~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int shortestSong(int A, int B) {
    if (A < B) {
        return A;
    } else {
        return B;
    }
}

/*~~~~~~~~~~~
Write samples
~~~~~~~~~~~~*/

bool write_sample(struct audio_t *audio_ptr, int sample) {
    if ((audio_ptr->wsrc > 0) && (audio_ptr->wslc > 0)) { // is there space available in both output FIFO's
        int out = sample << 12; // amplify signal
        audio_ptr->ldata = out; // write sample to output left
        audio_ptr->rdata = out; // write sample to output right
        return true; // if theres space then output true
    }
    return false; // if there isnt space then output false
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~
Mix vocals and instrumentals
~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int mixSong1(volatile audioState *audio) {
    int sample = 0; //start with sample = 0
    if (audio->song1VocalsOn) { //if vocals of song 1 are on add vocals of song 1
        sample = sample + vocalsA[audio->iA];
    }
    if (audio->song1InstrOn) { //if instrumantals of song 1 are on add instrumentals of song 1
        sample = sample + instrumentalA[audio->iA];
    }
    return sample;
}

int mixSong2(volatile audioState *audio) {
    int sample = 0;
    if (audio->song2VocalsOn) { //if vocals of song 1 are on add vocals of song 1
        sample = sample + vocalsB[audio->iB];
    }
    if (audio->song2InstrOn) { //if instrumantals of song 1 are on add instrumentals of song 1
        sample = sample + instrumentalB[audio->iB];
    }
    return sample;
}

//the update song functions determine when to go to the next index of the sample arrays and how much samples to skip
//this allows the BPM increase / BPM decrease to work
// when the BPM counter reaches 100 you update 1 sample
//the while loop at the starts acts as a tub that is filled based on if the sample is faster or solwer than 100
// for example if the numerator is greater than 100 we will have some overflow into the next run of the function causing it to skip an index
// if the numerator is smaller than 100 it will need a second function call to reach 100 and hence we will skip a sample
//this speeds up / slows the audio
// pressing the keys raises or lowers the numerator by 5
void updateSong1(volatile audioState *audio) {
    audio->bpmCounterA = audio->bpmCounterA + audio->bpmNumerator;

    while (audio->bpmCounterA >= audio->bpmDenominator) {
        audio->bpmCounterA = audio->bpmCounterA - audio->bpmDenominator;
        audio->iA++;

        if (audio->iA >= audio->songLenA) {
            audio->iA = 0;
        }
    }
}

void updateSong2(volatile audioState *audio) {
    audio->bpmCounterB = audio->bpmCounterB + audio->bpmNumerator;

    while (audio->bpmCounterB >= audio->bpmDenominator) {
        audio->bpmCounterB = audio->bpmCounterB - audio->bpmDenominator;
        audio->iB++;

        if (audio->iB >= audio->songLenB) {
            audio->iB = 0;
        }
    }
}

/*~~~~~~~~~~~~
Fader handeler
~~~~~~~~~~~~~~*/

int clip16(int x) {
    if (x > 32767)
        return 32767; //clamps signal to a max and min value
    if (x < -32768)
        return -32768;
    return x;
}

int fadeSample(volatile audioState *audio) {
    int sampleA;
    int sampleB;
    int mixed;
    sampleA = mixSong1(audio); //mixed samples
    sampleB = mixSong2(audio); //mixed samples

    if (audio->blockFade == 0) { // if no fade is happening
        if (audio->activeSong == 0) {
            return sampleA; //return sample A if activeSong is 0
        } else {
            return sampleB; //return sample B if activeSong is 0
        }
    }

    if (audio->fadeSongAtoB) { // fade from A to B
        mixed = (sampleA * (FADE_SAMPLES - audio->fadeCount) + sampleB * audio->fadeCount) / FADE_SAMPLES;
        // the output becomes (song A * time left in fade + B * time elapsed in fade) / total fade time
    } else { // fade from B to A
        mixed = (sampleB * (FADE_SAMPLES - audio->fadeCount) + sampleA * audio->fadeCount) / FADE_SAMPLES;
        // the output becomes (song A * time left in fade + B * time elapsed in fade) / total fade time
    }

    mixed = clip16(mixed); //stay within the 16bit range

    return mixed;
}

void updateFade(volatile audioState *audio) {
    if (audio->blockFade == 0) { //if fade off then play active song
        if (audio->activeSong == 0) {
            updateSong1(audio);
        } else {
            updateSong2(audio);
        }
        return;
    }
    // if fade on update both songs
    updateSong1(audio);
    updateSong2(audio);

    //increase fadecount
    audio->fadeCount++;

    if (audio->fadeCount >= FADE_SAMPLES) { //when fade is done

        if (audio->fadeSongAtoB) { //swap songs
            audio->activeSong = 1;
        } else if (audio->fadeSongBtoA) {
            audio->activeSong = 0;
        }
        audio->fadeSongAtoB = 0; //reset flags
        audio->fadeSongBtoA = 0;
        audio->blockFade = 0;
        audio->fadeCount = 0;
    }
}

void start_fade(volatile audioState *audio) { //function sets up a new fade with flags
    if (audio->blockFade == 1) {
        return;
    }

    audio->blockFade = 1;
    audio->fadeCount = 0;

    if (audio->activeSong == 0) {
        audio->fadeSongAtoB = 1;
        audio->fadeSongBtoA = 0;
        fade_arrow_target = 1;
    } else {
        audio->fadeSongAtoB = 0;
        audio->fadeSongBtoA = 1;
        fade_arrow_target = 0;
    }
}

/*~~~~~~~~~~~~
Echo function
~~~~~~~~~~~~~~*/

int apply_echo(volatile audioState *audio, int sample, int ring_buffer[]) {
    int delayed;
    int out;
    int fs;

    if (audio->echoOn == 0) { //if echo is off retuen regular sample
        return sample;
    }

    delayed = ring_buffer[audio->iEcho]; //take an old sample from buffer
    out = sample + ((delayed * 2) / 3); //add old sample to current sample
    out = clip16(out); //clip sample
    fs = sample + (delayed / 3);
    fs = clip16(fs);

    ring_buffer[audio->iEcho] = sample; //rewrite spot in buffer with current sample

    audio->iEcho++;
    if (audio->iEcho >= ECHO_DELAY) { //advance circular buffer
        audio->iEcho = 0;
    }

    return out;
}

/*~~~~~~~~~~~~
BPM functions
~~~~~~~~~~~~~~*/

void raiseBPM(volatile audioState *audio) { //increment BPM numerator
    if (audio->bpmNumerator < 130) {
        audio->bpmNumerator += 5;
    }
}

void lowerBPM(volatile audioState *audio) { //decrement BPM denominator
    if (audio->bpmNumerator > 70) {
        audio->bpmNumerator -= 5;
    }
}

//VGA FUnctions

void draw_arrow_left(void) {
    fill_rect(152, 118, 30, 4, RED);
    fill_rect(148, 112, 4, 16, RED);
    fill_rect(144, 114, 4, 12, RED);
    fill_rect(140, 116, 4, 8, RED);
    fill_rect(136, 118, 4, 4, RED);
}

void draw_arrow_right(void) {
    fill_rect(136, 118, 30, 4, RED);
    fill_rect(164, 112, 4, 16, RED);
    fill_rect(168, 114, 4, 12, RED);
    fill_rect(172, 116, 4, 8, RED);
    fill_rect(176, 118, 4, 4, RED);
}
void clear_arrow(void) {
    fill_rect(134, 110, 50, 20, DARK_GRAY);
}

void fill_button1(void) {
    fill_rect(46, 175, 13, 13, RED);
}

void fill_button2(void) {
    fill_rect(80, 175, 13, 13, RED);
}

void fill_button3(void) {
    fill_rect(129, 175, 13, 13, RED);
}

void fill_button4(void) {
    fill_rect(176, 175, 13, 13, RED);
}

void fill_button5(void) {
    fill_rect(228, 175, 13, 13, RED);
}

void fill_button6(void) {

    fill_rect(262, 175, 13, 13, RED);
}

void plot_pixel(int x, int y, uint16_t color) {
    *(volatile uint16_t *)((char *)pixel_buffer_start + (y << 10) + (x << 1)) = color;
}

void clear_screen(uint16_t color) {
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        for (int x = 0; x < SCREEN_WIDTH; x++) {
            plot_pixel(x, y, color);
        }
    }
}

void draw_image_centered(void) { // draws background
    int dst_y = (SCREEN_HEIGHT - DJ_BACKGROUND_HEIGHT) / 2;

    for (int y = 0; y < DJ_BACKGROUND_HEIGHT; y++) {
        for (int x = 0; x < DJ_BACKGROUND_WIDTH; x++) {
            uint16_t color = (uint16_t)DJ_Background[y * DJ_BACKGROUND_WIDTH + x];
            plot_pixel(x, dst_y + y, color);
        }
    }
}

void fill_rect(int x, int y, int width, int height, short int color) {
    int row, col;

    for (row = 0; row < height; row++) {
        for (col = 0; col < width; col++) {
            plot_pixel(x + col, y + row, color);
        }
    }
}

void fill_circle(int cx, int cy, int r, short int color) {
    int x, y;
    int r_sq = r * r;

    for (y = -r; y <= r; y++) {
        for (x = -r; x <= r; x++) {
            if ((x * x + y * y) <= r_sq) {
                plot_pixel(cx + x, cy + y, color);
            }
        }
    }
}

void wait_for_vsync(volatile int *pixel_ctrl_ptr) {
    *pixel_ctrl_ptr = 1;
    while (*(pixel_ctrl_ptr + 3) & 0x1)
        ;
}

void select_active_song_from_switches(volatile audioState *audio) {
    int left_on = audio->song1VocalsOn || audio->song1InstrOn;
    int right_on = audio->song2VocalsOn || audio->song2InstrOn;

    if (audio->blockFade)
        return;

    if (left_on && !right_on)
        audio->activeSong = 0;
    else if (right_on && !left_on)
        audio->activeSong = 1;
}

void clear_switch_boxes(void) {
    fill_rect(46, 175, 13, 13, DARK_GRAY);
    fill_rect(80, 175, 13, 13, DARK_GRAY);
    fill_rect(129, 175, 13, 13, DARK_GRAY);
    fill_rect(176, 175, 13, 13, DARK_GRAY);
    fill_rect(228, 175, 13, 13, DARK_GRAY);
    fill_rect(262, 175, 13, 13, DARK_GRAY);
}

void draw_switch_boxes(void) {
    clear_switch_boxes();

    if (vga_switch_mask & 0x01) {
        fill_button1();
    }
    if (vga_switch_mask & 0x02) {
        fill_button2();
    }
    if (vga_switch_mask & 0x04) {
        fill_button6();
    }
    if (vga_switch_mask & 0x08) {
        fill_button5();
    }
    if (vga_switch_mask & 0x10) {
        fill_button4();
    }
    if (vga_switch_mask & 0x40) {
        fill_button3();
    }
}

void draw_fade_arrow(int fade_arrow_target) {
    clear_arrow();

    if (fade_arrow_target == 0)
        draw_arrow_left();
    else
        draw_arrow_right();
}

void request_vga_frame(void) {
    vga_redraw_request = 1;
}

void init_vga_scene(volatile int *pixel_ctrl_ptr) {
    *(pixel_ctrl_ptr + 1) = (int)&Buffer1;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1);
    draw_image_centered();
    draw_switch_boxes();
    clear_arrow();

    wait_for_vsync(pixel_ctrl_ptr);

    *(pixel_ctrl_ptr + 1) = (int)&Buffer2;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1);
    draw_image_centered();
    draw_switch_boxes();
    clear_arrow();

    pixel_buffer_start = *(pixel_ctrl_ptr + 1);
}

void render_vga_frame(volatile int *pixel_ctrl_ptr) {
    vga_redraw_request = 0;

    pixel_buffer_start = *(pixel_ctrl_ptr + 1);

    draw_switch_boxes();

    if (audio.blockFade)
        draw_fade_arrow(fade_arrow_target);
    else
        clear_arrow();

    *pixel_ctrl_ptr = 1;
    vga_swap_pending = 1;
}
