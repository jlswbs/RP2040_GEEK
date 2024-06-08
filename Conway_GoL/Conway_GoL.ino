// Conway's game of life cellular automata //

#include "hardware/structs/rosc.h"
#include "st7789_lcd.pio.h"

#define PIN_DIN   11
#define PIN_CLK   10
#define PIN_CS    9
#define PIN_DC    8
#define PIN_RESET 12
#define PIN_BL    25

PIO pio = pio0;
uint sm = 0;
uint offset = pio_add_program(pio, &st7789_lcd_program);

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define WIDTH   120
#define HEIGHT  68
#define FULLW   240
#define FULLH   135
#define SCR     (FULLW*FULLH)
#define GENNUM  750

  uint16_t col[SCR];
  bool grid[2][WIDTH][HEIGHT];
  int current, gen;

#define SERIAL_CLK_DIV 1.f

static const uint8_t st7789_init_seq[] = {
  
  1, 20, 0x01,                        // Software reset
  1, 10, 0x11,                        // Exit sleep mode
  2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
  2, 0, 0x36, 0x70,                   // Set MADCTL: row then column, refresh is bottom to top ????
  5, 0, 0x2a, 0x00, 0x28, 0x01, 0x17, // CASET: column addresses from 0 to 240 (f0)
  5, 0, 0x2b, 0x00, 0x35, 0x00, 0xbb, // RASET: row addresses from 0 to 240 (f0)
  1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
  1, 2, 0x13,                         // Normal display on, then 10 ms delay
  1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
  0                                   // Terminate list

};

static inline void lcd_set_dc_cs(bool dc, bool cs) {

  sleep_us(1);
  gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
  sleep_us(1);

}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count) {

  st7789_lcd_wait_idle(pio, sm);
  lcd_set_dc_cs(0, 0);
  st7789_lcd_put(pio, sm, *cmd++);
  if (count >= 2) {
  st7789_lcd_wait_idle(pio, sm);
  lcd_set_dc_cs(1, 0);
  for (size_t i = 0; i < count - 1; ++i) st7789_lcd_put(pio, sm, *cmd++);
  }
  st7789_lcd_wait_idle(pio, sm);
  lcd_set_dc_cs(1, 1);

}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {

  const uint8_t *cmd = init_seq;
  while (*cmd) {
  lcd_write_cmd(pio, sm, cmd + 2, *cmd);
  sleep_ms(*(cmd + 1) * 5);
  cmd += *cmd + 2;
  }
}

static inline void st7789_start_pixels(PIO pio, uint sm) {

  uint8_t cmd = 0x2c; // RAMWR
  lcd_write_cmd(pio, sm, &cmd, 1);
  lcd_set_dc_cs(1, 0);

}

static inline void seed_random_from_rosc(){
  
  uint32_t random = 0;
  uint32_t random_bit;
  volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

  for (int k = 0; k < 32; k++) {
    while (1) {
      random_bit = (*rnd_reg) & 1;
      if (random_bit != ((*rnd_reg) & 1)) break;
    }

    random = (random << 1) | random_bit;
  }

  srand(random);
}
  
void rndseed(){

  memset(col,0,2 * SCR);

  current = 0;

  for (int x = 0; x < WIDTH; x++) {
    for (int y = 0; y < HEIGHT; y++) grid[0][x][y] = rand()%2;
  }

}

void RunGrid(){
  
  int x, y;
  int count;
  int value = 0;
  int new_grid;

  new_grid = 1 - current;
  
  for (y = 0; y < HEIGHT; y++) {
  
    for (x = 0; x < WIDTH; x++) {
      
      count = Neighbours(x, y);
      
      if (count < 2 || count > 3) { value = 0; }
      else if (count == 3) { value = 3; }
      else { value = grid[current][x][y]; }
    
      grid[new_grid][x][y] = value;

      if(grid[current][x][y]) col[(2*x)+(2*y)*FULLW] = WHITE;
      else col[(2*x)+(2*y)*FULLW] = BLACK;
    
    }
  }
  
  current = new_grid;
}


int Neighbours(int x, int y){
  
  int i, j;
  int result = 0;

  x--;
  y--;
  
  for (i = 0; i < 3; i++) {
  
    if (y < 0 || y > (HEIGHT - 1)) continue;

    for (j = 0; j < 3; j++) {
      if (x < 0 || x > (WIDTH - 1)) continue;
      if (i==1 && j == 1) { x++; continue; }
      if (grid[current][x][y]) result++;
      x++;
    }
    y++;
    x -= 3;
  }
  
  return result;
} 

void setup(){

  st7789_lcd_program_init(pio, sm, offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

  gpio_init(PIN_CS);
  gpio_init(PIN_DC);
  gpio_init(PIN_RESET);
  gpio_init(PIN_BL);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_set_dir(PIN_DC, GPIO_OUT);
  gpio_set_dir(PIN_RESET, GPIO_OUT);
  gpio_set_dir(PIN_BL, GPIO_OUT);

  gpio_put(PIN_CS, 1);
  gpio_put(PIN_RESET, 1);
  lcd_init(pio, sm, st7789_init_seq);
  gpio_put(PIN_BL, 1);

  seed_random_from_rosc();
  
  rndseed();
  
}


void loop(){
  
  st7789_start_pixels(pio, sm);

  RunGrid();

  for (int y = 0; y < FULLH; y++) {

    for (int x = 0; x < FULLW; x++) {

      uint16_t coll = col[x+y*FULLW];
      st7789_lcd_put(pio, sm, coll >> 8);
      st7789_lcd_put(pio, sm, coll & 0xff);

    }

  }

  if (gen == GENNUM) {

    gen = 0;
    rndseed();

  }

  gen++;
 
}