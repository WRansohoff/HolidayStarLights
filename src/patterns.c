#include "patterns.h"

// Get a 'cycling rainbow' color, given a maximum and 'progress' time
// value. The result is returned as a 32-bit int; 0xRRGGBB00
// TODO: There are faster ways to do this, but...meh. It's a one-off.
uint32_t rainbow_cycle( uint32_t prg, uint32_t max ) {
  // No color if the value is out of range.
  if ( prg > max ) { return 0x00000000; }
  uint8_t r, g, b;
  r = g = b = 0x00;
  int step = max / 6;
  // Red color.
  if ( ( ( prg > 0 ) && ( prg < step ) ) || ( prg > ( step * 5 ) ) ) {
    r = 0xFF;
  }
  else if ( ( prg > ( step * 2 ) && ( prg < ( step * 4 ) ) ) ) {
    r = 0x00;
  }
  else if ( prg < ( step * 2 ) ) {
    r = 0xFF - ( ( ( prg - step ) * 0xFF ) / step );
  }
  else {
    r = ( ( prg - ( step * 4 ) ) * 0xFF ) / step;
  }
  // Green color.
  if ( ( ( prg > step ) && ( prg < ( step * 3 ) ) ) ) {
    g = 0xFF;
  }
  else if ( ( prg > ( step * 4 ) ) ) {
    g = 0x00;
  }
  else if ( ( prg > ( step * 3 ) ) && ( prg < ( step * 4 ) ) ) {
    g = 0xFF - ( ( ( prg - ( step * 3 ) ) * 0xFF ) / step );
  }
  else {
    g = ( prg * 0xFF ) / step;
  }
  // Blue color.
  if ( ( prg > ( step * 3 ) && prg < ( step * 5 ) ) ) {
    b = 0xFF;
  }
  else if ( ( prg < ( step * 2 ) ) ) {
    b = 0x00;
  }
  else if ( ( prg > ( step * 5 ) ) ) {
    b = 0xFF - ( ( ( prg - ( step * 5 ) ) * 0xFF ) / step );
  }
  else {
    b = ( ( prg - ( step * 2 ) ) * 0xFF ) / step;
  }
  // Done, return the full color in one word.
  return ( uint32_t )( ( r << 24 ) | ( g << 16 ) | ( b << 8 ) );
}

// Get a 'cycling rainbow' color, given a maximum and 'progress' time
// value. The result is returned as a 32-bit int; 0xRRGGBB00
// This is a lower-power algorithm which does not hold colors at
// full brightness for long periods of time.
uint32_t rainbow_lp_cycle( uint32_t prg, uint32_t max ) {
  // No color if the value is out of range.
  if ( prg > max ) { return 0x00000000; }
  uint8_t r, g, b;
  r = g = b = 0x00;
  int step = max / 3;
  // Red color.
  if ( ( prg > step ) && ( prg < ( step * 2 ) ) ) {
    r = 0x00;
  }
  else if ( prg <= step ) {
    r = 0xFF - ( ( prg * 0xFF ) / step );
  }
  else {
    r = ( ( prg - ( step * 2 ) ) * 0xFF ) / step;
  }
  // Green color.
  if ( ( prg > ( step * 2 ) ) ) {
    g = 0x00;
  }
  else if ( prg > step ) {
    g = 0xFF - ( ( ( prg - step ) * 0xFF ) / step );
  }
  else {
    g = ( prg * 0xFF ) / step;
  }
  // Blue color.
  if ( prg < step ) {
    b = 0x00;
  }
  else if ( prg > ( step * 2 ) ) {
    b = 0xFF - ( ( ( prg - ( step * 2 ) ) * 0xFF ) / step );
  }
  else {
    b = ( ( prg - step ) * 0xFF ) / step;
  }
  // Done, return the full color in one word.
  return ( uint32_t )( ( r << 24 ) | ( g << 16 ) | ( b << 8 ) );
}

// Set a 24-byte GRB pixel color from 3 RGB bytes.
void set_px_rgb( uint8_t* px, uint8_t r, uint8_t g, uint8_t b ) {
  // Green color.
  for ( int i = 0; i < 8; ++i ) {
    if ( g & ( 1 << ( 7 - i ) ) ) { px[ i ] = WS2812_ON; }
    else { px[ i ] = WS2812_OFF; }
  }
  // Red color.
  for ( int i = 0; i < 8; ++i ) {
    if ( r & ( 1 << ( 7 - i ) ) ) { px[ i + 8 ] = WS2812_ON; }
    else { px[ i + 8 ] = WS2812_OFF; }
  }
  // Blue color.
  for ( int i = 0; i < 8; ++i ) {
    if ( b & ( 1 << ( 7 - i ) ) ) { px[ i + 16 ] = WS2812_ON; }
    else { px[ i + 16 ] = WS2812_OFF; }
  }
}

// Step one star's lighting pattern.
void step_star( star_t* star ) {
  // Use the same definition of 'now' for the whole function.
  int this_tick = systick;
  // Move to the next pattern if necessary.
  if ( star->next_step < this_tick ) {
    star->last_step = this_tick;
    star->next_step = this_tick + STEP_DUR;
    ++star->cur_pattern;
    if ( star->cur_pattern == ls_max ) { star->cur_pattern = ls_min; }
  }

  // Update colors to match the current pattern.
  // The downside of using an ancient STM32F1 chip is that it
  // lacks floating-point hardware, so integer math is much faster.
  uint8_t step_brightness = ( ( this_tick - star->last_step ) * 0xFF ) / ( STEP_DUR / 2 );
  if ( ( this_tick - star->last_step ) > ( STEP_DUR / 2 ) ) {
    step_brightness = 0xFF - ( step_brightness - 0xFF );
  }
  if ( star->cur_pattern == breathe_r ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), step_brightness, 0x00, 0x00 );
    }
  }
  else if ( star->cur_pattern == breathe_g ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, step_brightness, 0x00 );
    }
  }
  else if ( star->cur_pattern == breathe_b ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, 0x00, step_brightness );
    }
  }
  else if ( star->cur_pattern == xmas_odd ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      if ( i % 2 == 0 ) {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), step_brightness, 0x00, 0x00 );
      }
      else {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, step_brightness, 0x00 );
      }
    }
  }
  else if ( star->cur_pattern == xmas_even ) {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      if ( i % 2 == 1 ) {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), step_brightness, 0x00, 0x00 );
      }
      else {
        set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, step_brightness, 0x00 );
      }
    }
  }
  else if ( star->cur_pattern == rainbow ) {
    uint32_t r_step = STEP_DUR / STAR_LEDS;
    uint32_t r_prg = star->next_step - this_tick;
    for ( int i = 0; i < STAR_LEDS; ++i ) {
      uint32_t r_col = rainbow_cycle( r_prg, STEP_DUR );
      set_px_rgb( &( star->my_colors[ i * 24 ] ),
                  ( ( r_col >> 24 ) & 0xFF ),
                  ( ( r_col >> 16 ) & 0xFF ),
                  ( ( r_col >>  8 ) & 0xFF ) );
      r_prg += r_step;
      if ( r_prg > STEP_DUR ) { r_prg -= STEP_DUR; }
    }
  }
  else if ( star->cur_pattern == rainbow_lp ) {
    uint32_t r_step = STEP_DUR / STAR_LEDS;
    uint32_t r_prg = star->next_step - this_tick;
    for ( int i = 0; i < STAR_LEDS; ++i ) {
      uint32_t r_col = rainbow_lp_cycle( r_prg, STEP_DUR );
      set_px_rgb( &( star->my_colors[ i * 24 ] ),
                  ( ( r_col >> 24 ) & 0xFF ),
                  ( ( r_col >> 16 ) & 0xFF ),
                  ( ( r_col >>  8 ) & 0xFF ) );
      r_prg += r_step;
      if ( r_prg > STEP_DUR ) { r_prg -= STEP_DUR; }
    }
  }
  else {
    for ( int i = 0; i < STAR_LEDS; ++i )  {
      set_px_rgb( &( star->my_colors[ i * 24 ] ), 0x00, 0x00, 0x00 );
    }
  }
}
